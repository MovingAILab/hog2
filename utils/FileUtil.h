/*
 *  $Id: FileUtil.h
 *  hog2
 *
 *  Created by Junwen Shen on 4/25/23.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef HOG2_FILEUTIL_H
#define HOG2_FILEUTIL_H

#include <string>

/**
 * Check if a file exists
 * @see https://e-penguiner.com/cpp-function-check-file-exist/
 */
bool FileExists(const std::string &filename);

#endif  // HOG2_FILEUTIL_H
