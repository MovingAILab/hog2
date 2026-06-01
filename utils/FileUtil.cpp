/*
 *  $Id: FileUtil.cpp
 *  hog2
 *
 *  Created by Junwen Shen on 4/25/23.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "FileUtil.h"

#if __cplusplus < 201703L  // C++17

#include <sys/stat.h>

// Windows doesn't have S_ISDIR or S_ISREG
#ifndef S_ISDIR
#define S_ISDIR(mode) (((mode) & S_IFMT) == S_IFDIR)
#endif
#ifndef S_ISREG
#define S_ISREG(mode) (((mode) & S_IFMT) == S_IFREG)
#endif

#else
#include <filesystem>
#endif

bool FileExists(const std::string &filename) {
#if __cplusplus < 201703L
    struct stat buffer{};
    return stat(filename.c_str(), &buffer) == 0 && S_ISREG(buffer.st_mode);
#else
    return std::filesystem::is_regular_file(filename);
#endif
}
