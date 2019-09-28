#include "line_extractor/line_extractor.h"
#include <list>
#include <math.h>
// line extractor base on split algorithm

const int MINIMUM_POINTS_CHECK = 2;
const int MINIMUM_INDEX = 2;
const double MAXIMUM_GAP_DISTANCE = 0.2;
const double IN_LINE_THRESHOLD = 0.1;

void lineExtractor(const std::vector<Point> data,
                   std::vector<Line>& real_line) {

  if (data.size() < MINIMUM_POINTS_CHECK) return;

  // 1#: we initial a line from start to end
  //-----------------------------------------
  Point start = data.front();
  Point end = data.back();
  Line l;
  l.start_index = 0;
  l.end_index = data.size() - 1;
  std::list<Line> line_list;
  line_list.push_back(l);

  while (!line_list.empty()) {
    // 2#: everytime we take the first line in
    //line list to check if every point is on this line
    //-----------------------------------------
    Line& lr = *line_list.begin();

    //
    if (lr.end_index - lr.start_index < MINIMUM_INDEX || lr.start_index == lr.end_index)
    {
      line_list.pop_front();
      continue;
    }

    // 3#: use two points to generate a line equation
    //-----------------------------------------
    start.x = data[lr.start_index].x;
    start.y = data[lr.start_index].y;
    end.x = data[lr.end_index].x;
    end.y = data[lr.end_index].y;

    // two points P1(x1, y1), P2(x2,y2) are given, and these two points are not the same
    // we can calculate an equation to model a line these two points are on.
    // A = y2 - y1
    // B = x1 - x2
    // C = x2 * y1 - x1 * y2
    double A = end.y - start.y;
    double B = start.x - end.x;
    double C = end.x * start.y - start.x * end.y;

    double max_distance = 0;
    int max_i;
    int gap_i(-1);
    // the kernel code
    for (int i = lr.start_index + 1; i <= lr.end_index - 1; i++) {
      // 4#: if two points' distance is too large, it's meaningless to generate a line
      // connects these two points, so we have to filter it.
      //-----------------------------------------
      double point_gap_dist = hypot(data[i].x - data[i+1].x, data[i].y - data[i+1].y);
      if (point_gap_dist > MAXIMUM_GAP_DISTANCE) {
        gap_i = i;
        break;
      }

      // 5#: calculate the distance between every point to the line
      //-----------------------------------------
      double dist = fabs(A * data[i].x + B * data[i].y + C) / hypot(A, B);
      if (dist > max_distance) {
        max_distance = dist;
        max_i = i;
      }
    }

    // 6#: if gap is too large or there's a point is far from the line,
    // we have to split this line to two line, then check again.
    //-----------------------------------------
    if (gap_i != -1) {
      int tmp = lr.end_index;
      lr.end_index = gap_i;
      Line ll;
      ll.start_index = gap_i + 1;
      ll.end_index = tmp;
      line_list.insert(++line_list.begin(), ll);
    }
    else if (max_distance > IN_LINE_THRESHOLD) {
      int tmp = lr.end_index;
      lr.end_index = max_i;
      Line ll;
      ll.start_index = max_i + 1;
      ll.end_index = tmp;
      line_list.insert(++line_list.begin(), ll);
    } else {
      real_line.push_back(line_list.front());
      line_list.pop_front();
    }
  }
}
