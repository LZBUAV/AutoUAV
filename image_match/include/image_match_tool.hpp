#ifndef IMAGE_MATCH_TOOL_HPP
#define IMAGE_MATCH_TOOL_HPP

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <string.h>

std::vector<std::string> readFolder(const std::string &image_path);

unsigned long GetTickCount();

int split_imagename(const std::string &name, const char& delim);

bool cmp(std::string a, std::string b);
#endif