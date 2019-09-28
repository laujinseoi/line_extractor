#ifndef LINE_EXTRACTOR_H
#define LINE_EXTRACTOR_H
#include "line.h"
#include "point.h"
#include <vector>

void lineExtractor(const std::vector<Point> data,
                   std::vector<Line>& real_line);
#endif // LINE_EXTRACTOR_H
