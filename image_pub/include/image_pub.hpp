#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <string.h>

std::vector<std::string> readFolder(const std::string &image_path)
{
    std::vector<std::string> image_names;
    auto dir = opendir(image_path.c_str());

    if ((dir) != nullptr)
    {
        struct dirent *entry;
        entry = readdir(dir);
        while (entry)
        {
            auto temp = image_path + "/" + entry->d_name;
            if (strcmp(entry->d_name, "") == 0 || strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            {
                entry = readdir(dir);
                continue;
            }
            image_names.push_back(temp);
            entry = readdir(dir);
        }
    }
    return image_names;
}

unsigned long GetTickCount()
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

}

int split_imagename(const std::string &name, const char& delim)
{
    std::stringstream iss(name);
    std::string item;
    std::vector<std::string> items;
    while(std::getline(iss, item, delim))
    {
        items.push_back(item);
    }
    // for(auto i : items)
    // {
    //     std::cout << i << ", ";
    // }
    // std::cout << std::endl;
    return std::stoi(items[3]);
}

bool cmp(std::string a, std::string b)
{
    return split_imagename(a, '_') < split_imagename(b, '_');
}