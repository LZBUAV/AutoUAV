#include "nanodet.h"
#include "yaml-cpp/yaml.h"
#include "common.hpp"


nanodet::nanodet(const std::string &config_file) {
    YAML::Node root = YAML::LoadFile(config_file);
    YAML::Node config = root["nanodet"];
    onnx_file = config["onnx_file"].as<std::string>();
    engine_file = config["engine_file"].as<std::string>();
    labels_file = config["labels_file"].as<std::string>();
    BATCH_SIZE = config["BATCH_SIZE"].as<int>();
    INPUT_CHANNEL = config["INPUT_CHANNEL"].as<int>();
    IMAGE_WIDTH = config["IMAGE_WIDTH"].as<int>();
    IMAGE_HEIGHT = config["IMAGE_HEIGHT"].as<int>();
    img_mean = config["img_mean"].as<std::vector<float>>();
    img_std = config["img_mean"].as<std::vector<float>>();
    obj_threshold = config["obj_threshold"].as<float>();
    nms_threshold = config["nms_threshold"].as<float>();
    strides = config["strides"].as<std::vector<int>>();
    detect_labels = readCOCOLabel(labels_file);
    CATEGORY = detect_labels.size();
    class_colors.resize(CATEGORY);
    refer_rows = 0;
    refer_cols = 3;
    for (const int &stride : strides) {
        refer_rows += IMAGE_WIDTH * IMAGE_HEIGHT / stride / stride;
    }
    GenerateReferMatrix();
    srand((int) time(nullptr));
    for (cv::Scalar &class_color : class_colors)
        class_color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
}

nanodet::~nanodet() = default;

void nanodet::LoadEngine() {
    // create and load engine
    std::fstream existEngine;
    existEngine.open(engine_file, std::ios::in);
    if (existEngine) {
        readTrtFile(engine_file, engine);
        assert(engine != nullptr);
    } else {
        onnxToTRTModel(onnx_file, engine_file, engine, BATCH_SIZE);
        assert(engine != nullptr);
    }
}

bool nanodet::InferenceFolder(const std::string &folder_name) {
    std::vector<std::string> sample_images = readFolder(folder_name);
    //get context
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);

    //get buffers
    assert(engine->getNbBindings() == 2);
    
    int nbBindings = engine->getNbBindings();
    bufferSize.resize(nbBindings);

    for (int i = 0; i < nbBindings; ++i) {
        nvinfer1::Dims dims = engine->getBindingDimensions(i);
        nvinfer1::DataType dtype = engine->getBindingDataType(i);
        int64_t totalSize = volume(dims) * 1 * getElementSize(dtype);
        bufferSize[i] = totalSize;
        std::cout << "binding" << i << ": " << totalSize << std::endl;
        cudaMalloc(&buffers[i], totalSize);
    }

    //get stream
    
    cudaStreamCreate(&stream);

    outSize = int(bufferSize[1] / sizeof(float) / BATCH_SIZE);

    EngineInference(sample_images, outSize, buffers, bufferSize, stream);

    // release the stream and the buffers
    cudaStreamDestroy(stream);
    cudaFree(buffers[0]);
    cudaFree(buffers[1]);

    // destroy the engine
    context->destroy();
    engine->destroy();
}
bool nanodet::CreateContext() {
    //get context
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);

    //get buffers
    assert(engine->getNbBindings() == 2);
    int nbBindings = engine->getNbBindings();
    bufferSize.resize(nbBindings);

    for (int i = 0; i < nbBindings; ++i) {
        nvinfer1::Dims dims = engine->getBindingDimensions(i);
        nvinfer1::DataType dtype = engine->getBindingDataType(i);
        int64_t totalSize = volume(dims) * 1 * getElementSize(dtype);
        bufferSize[i] = totalSize;
        std::cout << "binding" << i << ": " << totalSize << std::endl;
        cudaMalloc(&buffers[i], totalSize);
    }

    //get stream
    cudaStreamCreate(&stream);

    outSize = int(bufferSize[1] / sizeof(float) / BATCH_SIZE);

}
void nanodet::EngineInference(const std::vector<std::string> &image_list, const int &outSize, void **buffers,
                                 const std::vector<int64_t> &bufferSize, cudaStream_t stream) {
    int index = 0;
    int batch_id = 0;
    std::vector<cv::Mat> vec_Mat(BATCH_SIZE);
    std::vector<std::string> vec_name(BATCH_SIZE);
    cv::Mat face_feature(image_list.size(), outSize, CV_32FC1);
    float total_time = 0;
    for (const std::string &image_name : image_list)
    {
        index++;
        std::cout << "Processing: " << image_name << std::endl;
        cv::Mat src_img = cv::imread(image_name);
        if (src_img.data)
        {
//            cv::cvtColor(src_img, src_img, cv::COLOR_BGR2RGB);
            vec_Mat[batch_id] = src_img.clone();
            vec_name[batch_id] = image_name;
            batch_id++;
        }
        if (batch_id == BATCH_SIZE or index == image_list.size())
        {
            auto t_start_pre = std::chrono::high_resolution_clock::now();
            std::cout << "prepareImage" << std::endl;
            std::vector<float>curInput = prepareImage(vec_Mat);
            auto t_end_pre = std::chrono::high_resolution_clock::now();
            float total_pre = std::chrono::duration<float, std::milli>(t_end_pre - t_start_pre).count();
            std::cout << "prepare image take: " << total_pre << " ms." << std::endl;
            total_time += total_pre;
            batch_id = 0;
            if (!curInput.data()) {
                std::cout << "prepare images ERROR!" << std::endl;
                continue;
            }
            // DMA the input to the GPU,  execute the batch asynchronously, and DMA it back:
            std::cout << "host2device" << std::endl;
            cudaMemcpyAsync(buffers[0], curInput.data(), bufferSize[0], cudaMemcpyHostToDevice, stream);

            // do inference
            std::cout << "execute" << std::endl;
            auto t_start = std::chrono::high_resolution_clock::now();
            context->execute(BATCH_SIZE, buffers);
            auto t_end = std::chrono::high_resolution_clock::now();
            float total_inf = std::chrono::duration<float, std::milli>(t_end - t_start).count();
            std::cout << "Inference take: " << total_inf << " ms." << std::endl;
            total_time += total_inf;
            std::cout << "execute success" << std::endl;
            std::cout << "device2host" << std::endl;
            std::cout << "post process" << std::endl;
            auto r_start = std::chrono::high_resolution_clock::now();
            auto *out = new float[outSize * BATCH_SIZE];
            cudaMemcpyAsync(out, buffers[1], bufferSize[1], cudaMemcpyDeviceToHost, stream);
            cudaStreamSynchronize(stream);
            auto results = postProcess(vec_Mat, out, outSize);
            delete[] out;
            auto r_end = std::chrono::high_resolution_clock::now();
            float total_res = std::chrono::duration<float, std::milli>(r_end - r_start).count();
            std::cout << "Post process take: " << total_res << " ms." << std::endl;
            for (int i = 0; i < (int)vec_Mat.size(); i++)
            {
                auto org_img = vec_Mat[i];
                if (!org_img.data)
                    continue;
                auto rects = results[i];
//                cv::cvtColor(org_img, org_img, cv::COLOR_BGR2RGB);
                for(const auto &rect : rects)
                {
                    char t[256];
                    sprintf(t, "%.2f", rect.prob);
                    std::string name = detect_labels[rect.classes] + "-" + t;
                    cv::putText(org_img, name, cv::Point(rect.x - rect.w / 2, rect.y - rect.h / 2 - 5), cv::FONT_HERSHEY_COMPLEX, 0.7, class_colors[rect.classes], 2);
                    cv::Rect rst(rect.x - rect.w / 2, rect.y - rect.h / 2, rect.w, rect.h);
                    cv::rectangle(org_img, rst, class_colors[rect.classes], 2, cv::LINE_8, 0);                }
                int pos = vec_name[i].find_last_of(".");
                std::string rst_name = vec_name[i].insert(pos, "_");
                std::cout << rst_name << std::endl;
                cv::imwrite(rst_name, org_img);
            }
            total_time += total_res;
            vec_Mat = std::vector<cv::Mat>(BATCH_SIZE);
        }
    }
    std::cout << "Average processing time is " << total_time / image_list.size() << "ms" << std::endl;
}

void nanodet::EngineInferenceOnce(cv::Mat &src_img, darknet_ros_msgs::BoundingBoxes &detect_results) {
    int index = 0;
    int batch_id = 0;
    std::vector<cv::Mat> vec_Mat(BATCH_SIZE);
    std::vector<std::string> vec_name(BATCH_SIZE);
    darknet_ros_msgs::BoundingBox detect_result;

    float total_time = 0;
  
    index++;
    // std::cout << "Processing: " << std::endl;
    // std::cout << src_img.size() << std::endl;
    if (src_img.data)
    {
        vec_Mat[batch_id] = src_img;
        vec_name[batch_id] = "a_nwe_image.ipg";
        batch_id++;
    }
    if (batch_id == BATCH_SIZE or index == 1)
    {
        auto t_start_pre = std::chrono::high_resolution_clock::now();
        // std::cout << "prepareImage" << std::endl;
        std::vector<float>curInput = prepareImage(vec_Mat);
        auto t_end_pre = std::chrono::high_resolution_clock::now();
        float total_pre = std::chrono::duration<float, std::milli>(t_end_pre - t_start_pre).count();
        // std::cout << "prepare image take: " << total_pre << " ms." << std::endl;
        total_time += total_pre;
        batch_id = 0;
        if (!curInput.data()) {
            std::cout << "prepare images ERROR!" << std::endl;
        }
        // DMA the input to the GPU,  execute the batch asynchronously, and DMA it back:
        // std::cout << "host2device" << std::endl;
        cudaMemcpyAsync(buffers[0], curInput.data(), bufferSize[0], cudaMemcpyHostToDevice, stream);

        // do inference
        // std::cout << "execute" << std::endl;
        auto t_start = std::chrono::high_resolution_clock::now();
        context->execute(BATCH_SIZE, buffers);
        auto t_end = std::chrono::high_resolution_clock::now();
        float total_inf = std::chrono::duration<float, std::milli>(t_end - t_start).count();
        // std::cout << "Inference take: " << total_inf << " ms." << std::endl;
        total_time += total_inf;
        // std::cout << "execute success" << std::endl;
        // std::cout << "device2host" << std::endl;
        // std::cout << "post process" << std::endl;
        auto r_start = std::chrono::high_resolution_clock::now();
        auto *out = new float[outSize * BATCH_SIZE];
        cudaMemcpyAsync(out, buffers[1], bufferSize[1], cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);
        auto results = postProcess(vec_Mat, out, outSize);
        delete[] out;
        auto r_end = std::chrono::high_resolution_clock::now();
        float total_res = std::chrono::duration<float, std::milli>(r_end - r_start).count();
        // std::cout << "Post process take: " << total_res << " ms." << std::endl;
        for (int i = 0; i < (int)vec_Mat.size(); i++)
        {
            auto org_img = vec_Mat[i];
            if (!org_img.data)
                continue;
            auto rects = results[i];
//                cv::cvtColor(org_img, org_img, cv::COLOR_BGR2RGB);
            for(const auto &rect : rects)
            {
                detect_result.id = rect.classes;
                detect_result.probability = rect.prob;
                detect_result.xmin = std::max(int(rect.x - rect.w / 2), 0);
                detect_result.xmax = std::min(int(rect.x + rect.w / 2), org_img.cols);
                detect_result.ymin = std::max(int(rect.y - rect.h / 2), 0);
                detect_result.ymax = std::min(int(rect.y + rect.h / 2), org_img.rows);
                detect_results.bounding_boxes.push_back(detect_result);

                char t[256];
                sprintf(t, "%.2f", rect.prob);
                std::string name = detect_labels[rect.classes] + "-" + t;
                cv::putText(org_img, name, cv::Point(rect.x - rect.w / 2, rect.y - rect.h / 2 - 5), cv::FONT_HERSHEY_COMPLEX, 0.7, class_colors[rect.classes], 2);
                cv::Rect rst(rect.x - rect.w / 2, rect.y - rect.h / 2, rect.w, rect.h);
                cv::rectangle(org_img, rst, class_colors[rect.classes], 2, cv::LINE_8, 0);  
                std::cout << "nanodet: " << rst.x << " " << rst.y << " " << rst.width << " " << rst.height << std::endl;  
                std::cout << "darknet: " << detect_result.xmin << " " << detect_result.ymin << " " << detect_result.xmax-detect_result.xmin << " " << detect_result.ymax-detect_result.ymin << std::endl;            
                std::cout << "--------------------" << std::endl;
            }
            // int pos = vec_name[i].find_last_of(".");
            // std::string rst_name = vec_name[i].insert(pos, "_");
            // std::cout << rst_name << std::endl;
            // cv::imwrite(rst_name, org_img);
        }
        total_time += total_res;
        vec_Mat = std::vector<cv::Mat>(BATCH_SIZE);
    }
    
    // std::cout << "processing time is " << total_time << "ms" << std::endl;
}
void nanodet::ReleaseTRT()
{
    // release the stream and the buffers
    cudaStreamDestroy(stream);
    cudaFree(buffers[0]);
    cudaFree(buffers[1]);

    // destroy the engine
    context->destroy();
    engine->destroy();
}
void nanodet::GenerateReferMatrix() {
    int index = 0;
    refer_matrix = cv::Mat(refer_rows, refer_cols, CV_32FC1);
    for (const int &stride : strides) {
        for (int h = 0; h < IMAGE_HEIGHT / stride; h++)
            for (int w = 0; w < IMAGE_WIDTH / stride; w++) {
                auto *row = refer_matrix.ptr<float>(index);
                row[0] = float((2 * w + 1) * stride - 1) / 2;
                row[1] = float((2 * h + 1) * stride - 1) / 2;
                row[2] = stride;
                index += 1;
            }
    }
}

std::vector<float> nanodet::prepareImage(std::vector<cv::Mat> &vec_img) {
    std::vector<float> result(BATCH_SIZE * IMAGE_WIDTH * IMAGE_HEIGHT * INPUT_CHANNEL);
    float *data = result.data();
    int index = 0;
    for (const cv::Mat &src_img : vec_img)
    {
        if (!src_img.data)
            continue;
        float ratio = float(IMAGE_WIDTH) / float(src_img.cols) < float(IMAGE_HEIGHT) / float(src_img.rows) ? float(IMAGE_WIDTH) / float(src_img.cols) : float(IMAGE_HEIGHT) / float(src_img.rows);
        cv::Mat flt_img = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);
        cv::Mat rsz_img;
        cv::resize(src_img, rsz_img, cv::Size(), ratio, ratio);
        rsz_img.copyTo(flt_img(cv::Rect(0, 0, rsz_img.cols, rsz_img.rows)));
        flt_img.convertTo(flt_img, CV_32FC3);

        //HWC TO CHW
        int channelLength = IMAGE_WIDTH * IMAGE_HEIGHT;
        std::vector<cv::Mat> split_img = {
                cv::Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_32FC1, data + channelLength * (index + 2)),
                cv::Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_32FC1, data + channelLength * (index + 1)),
                cv::Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_32FC1, data + channelLength * index)
        };
        index += 3;
        cv::split(flt_img, split_img);
        for (int i = 0; i < INPUT_CHANNEL; i++) {
            split_img[i] = (split_img[i] - img_mean[i]) / img_std[i];
        }
    }
    return result;
}

std::vector<std::vector<nanodet::DetectRes>> nanodet::postProcess(const std::vector<cv::Mat> &vec_Mat,
        float *output, const int &outSize) {
    std::vector<std::vector<DetectRes>> vec_result;
    int index = 0;
    for (const cv::Mat &src_img : vec_Mat)
    {
        std::vector<DetectRes> result;
        float *out = output + index * outSize;
        float ratio = std::max(float(src_img.cols) / float(IMAGE_WIDTH), float(src_img.rows) / float(IMAGE_HEIGHT));
        cv::Mat result_matrix = cv::Mat(refer_rows, CATEGORY + 4, CV_32FC1, out);
        for (int row_num = 0; row_num < refer_rows; row_num++) {
            DetectRes box;
            auto *row = result_matrix.ptr<float>(row_num);
            auto max_pos = std::max_element(row + 4, row + CATEGORY + 4);
            box.prob = row[max_pos - row];
            if (box.prob < obj_threshold)
                continue;
            box.classes = max_pos - row - 4;
            auto *anchor = refer_matrix.ptr<float>(row_num);
            box.x = (anchor[0] - row[0] * anchor[2] + anchor[0] + row[2] * anchor[2]) / 2 * ratio;
            box.y = (anchor[1] - row[1] * anchor[2] + anchor[1] + row[3] * anchor[2]) / 2 * ratio;
            box.w = (row[2] + row[0]) * anchor[2] * ratio;
            box.h = (row[3] + row[1]) * anchor[2] * ratio;
            result.push_back(box);
        }
        NmsDetect(result);
        vec_result.push_back(result);
        index++;
    }
    return vec_result;
}

void nanodet::NmsDetect(std::vector<DetectRes> &detections) {
    sort(detections.begin(), detections.end(), [=](const DetectRes &left, const DetectRes &right) {
        return left.prob > right.prob;
    });

    for (int i = 0; i < (int)detections.size(); i++)
        for (int j = i + 1; j < (int)detections.size(); j++)
        {
            float iou = IOUCalculate(detections[i], detections[j]);
            if (iou > nms_threshold)
                detections[j].prob = 0;
        }

    detections.erase(std::remove_if(detections.begin(), detections.end(), [](const DetectRes &det)
    { return det.prob == 0; }), detections.end());
}

float nanodet::IOUCalculate(const nanodet::DetectRes &det_a, const nanodet::DetectRes &det_b) {
    cv::Point2f center_a(det_a.x, det_a.y);
    cv::Point2f center_b(det_b.x, det_b.y);
    cv::Point2f left_up(std::min(det_a.x - det_a.w / 2, det_b.x - det_b.w / 2),
                        std::min(det_a.y - det_a.h / 2, det_b.y - det_b.h / 2));
    cv::Point2f right_down(std::max(det_a.x + det_a.w / 2, det_b.x + det_b.w / 2),
                           std::max(det_a.y + det_a.h / 2, det_b.y + det_b.h / 2));
    float distance_d = (center_a - center_b).x * (center_a - center_b).x + (center_a - center_b).y * (center_a - center_b).y;
    float distance_c = (left_up - right_down).x * (left_up - right_down).x + (left_up - right_down).y * (left_up - right_down).y;
    float inter_l = det_a.x - det_a.w / 2 > det_b.x - det_b.w / 2 ? det_a.x - det_a.w / 2 : det_b.x - det_b.w / 2;
    float inter_t = det_a.y - det_a.h / 2 > det_b.y - det_b.h / 2 ? det_a.y - det_a.h / 2 : det_b.y - det_b.h / 2;
    float inter_r = det_a.x + det_a.w / 2 < det_b.x + det_b.w / 2 ? det_a.x + det_a.w / 2 : det_b.x + det_b.w / 2;
    float inter_b = det_a.y + det_a.h / 2 < det_b.y + det_b.h / 2 ? det_a.y + det_a.h / 2 : det_b.y + det_b.h / 2;
    if (inter_b < inter_t || inter_r < inter_l)
        return 0;
    float inter_area = (inter_b - inter_t) * (inter_r - inter_l);
    float union_area = det_a.w * det_a.h + det_b.w * det_b.h - inter_area;
    if (union_area == 0)
        return 0;
    else
        return inter_area / union_area - distance_d / distance_c;
}
