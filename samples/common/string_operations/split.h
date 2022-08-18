#ifndef __SPLIT_H
#define __SPLIT_H

// functions to split a std::string by a specific delimiter
#include <string>
#include <vector>

void split(std::string& s, std::vector<std::string>& tokens, const std::string delimiter = " ")
{
    // std::cout << "[split DEBUG]delimiter: " << delimiter << " len:" << delimiter.length() << std::endl;
    std::string::size_type lastPos = 0;
    std::string::size_type pos = s.find(delimiter, lastPos);
    // if (0 == static_cast<int>(pos)) {
    // 	lastPos = delimiter.length();
    // 	pos = s.find(delimiter, lastPos);
    // }
    while (std::string::npos != pos && lastPos < s.length()) {
        // std::cout << "[split DEBUG]lastPos: " << lastPos << " pos: " << pos << std::endl;
        tokens.push_back(s.substr(lastPos, pos - lastPos));
        lastPos = pos + delimiter.length();
        pos = s.find(delimiter, lastPos);
    }
    if (s.length() != lastPos) {
        tokens.push_back(s.substr(lastPos));
        // std::cout << "[split DEBUG]lastPos: " << lastPos << " pos: " << s.length() << std::endl;
    }
}

#endif
