#include "location/amcl/weighted_areas.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
namespace amcl {

WeightedAreas::WeightedAreas(const std::string file_path) : file_path_(file_path) {
    // 读取权重区域信息
    ReadAreasFromJson(file_path_);
}

WeightedAreas::~WeightedAreas() {}

/**
 * @brief 从地图json文件读取权重区域信息
 * 
 * @param file_path，地图文件路径
 * @return boundingBoxs_，权重区域信息
 */
void WeightedAreas::ReadAreasFromJson(const std::string file_path) {
    Json::Reader reader;
    Json::Value root;
    
    std::ifstream infile(file_path);

    if(reader.parse(infile, root)) {
		// 读取 bounding_box 信息
		for (unsigned int i = 0; i < root["weighted_areas"].size(); i++)
		{
            boundingBox bounding_box;
            size_t vert_num;
			std::string box_type = root["weighted_areas"][i]["box_type"].asString();
            if (box_type.find("polygon") != std::string::npos) {
                bounding_box.boxtype = POLYGON;
                vert_num = root["weighted_areas"][i]["bounding_box"].size();
                bounding_box.vert_num = vert_num;
                bounding_box.corners = new double[vert_num][2];
                for (unsigned int j = 0; j < vert_num; j++) {
                    bounding_box.corners[j][0] = root["weighted_areas"][i]["bounding_box"][j]["x"].asDouble();
                    bounding_box.corners[j][1] = root["weighted_areas"][i]["bounding_box"][j]["y"].asDouble();
                }
            } else if (box_type.find("rectangle") != std::string::npos) {
                bounding_box.boxtype = RECTANGLE;
                vert_num = root["weighted_areas"][i]["bounding_box"].size();
                if (2 == vert_num) {
                    bounding_box.vert_num = vert_num;
                    bounding_box.corners = new double[vert_num][2];
                    for (unsigned int j = 0; j < vert_num; j++) {
                        bounding_box.corners[j][0] = root["weighted_areas"][i]["bounding_box"][j]["x"].asDouble();
                        bounding_box.corners[j][1] = root["weighted_areas"][i]["bounding_box"][j]["y"].asDouble();
                    }
                } else {
                    // SLAM_ERROR("rectangle error!");
                    continue;
                }
            } else if (box_type.find("circle") != std::string::npos) {
                bounding_box.boxtype = CIRCLE;
                vert_num = root["weighted_areas"][i]["bounding_box"].size();
                if (2 == vert_num) {
                    bounding_box.vert_num = vert_num;
                    bounding_box.corners = new double[2][2];
                    for (unsigned int j = 0; j < vert_num; j++) {
                        bounding_box.corners[j][0] = root["weighted_areas"][i]["bounding_box"][j]["x"].asDouble();
                        bounding_box.corners[j][1] = root["weighted_areas"][i]["bounding_box"][j]["y"].asDouble();
                    }
                } else {
                    // SLAM_ERROR("circle error!");
                    continue;
                }
            }
            bounding_box.weight = root["weighted_areas"][i]["weight"].asDouble();
            // bounding_box.mode = root["weighted_areas"][i]["mode"].asBool();
            m_boundingBoxs.push_back(bounding_box);
            std::cout << "bounding_box_" << i << " (box_type/weight/corners:" << box_type << "/" << bounding_box.weight
                 << "/" << bounding_box.vert_num << ")" << std::endl;
		}
        // std::cout << "bounding_box数量：" << m_boundingBoxs.size() << std::endl;
   }
}

/**
 * @brief 获取grid的权重
 * 
 * @param grid坐标
 * @return 该grid权重
 */
double WeightedAreas::GetWeightOfGrid(const double point[2]) {
    double weight = 1.0;
    if (!m_boundingBoxs.empty()) {
        for (auto box:m_boundingBoxs) {
            if (IsPointInBox(point, box)){
                weight = box.weight;
                return weight;
            }
        }
    }
    return weight;
}

/**
 * @brief 检查该grid是否在某区域内
 * 
 * @param grid坐标，边界框
 * @return false：不在区域内；true：在区域内
 */
bool WeightedAreas::IsPointInBox(const double point[2], const boundingBox box) {
    bool is_grid_in_box = false;
    // size_t corners_num = box.vert_num;
    // std::cout << "当前box角点数量：" << corners_num << std::endl;

    double x_dist, y_dist, center_distance;
    double center_x, center_y, radius_x, radius_y;
    double min_x, max_x, min_y, max_y;

    switch (box.boxtype) {
        case POLYGON:
            if (pnpoly(box.vert_num, point, box.corners)) {
                is_grid_in_box = true;
            }
            break;
        case CIRCLE:
            // 椭圆中心
            center_x = (box.corners[0][0] + box.corners[1][0]) / 2;
            center_y = (box.corners[0][1] + box.corners[1][1]) / 2;
            // 椭圆长短轴
            radius_x = fabs(box.corners[0][0] - box.corners[1][0]) / 2;
            radius_y = fabs(box.corners[0][1] - box.corners[1][1]) / 2;
            x_dist = point[0] - center_x;
            y_dist = point[1] - center_y;
            center_distance = (x_dist * x_dist) / (radius_x * radius_x)
                            + (y_dist * y_dist) / (radius_y * radius_y);
            if (center_distance < 1) {
                is_grid_in_box = true;
            }
            break;
        case RECTANGLE:
            min_x = std::min({box.corners[0][0], box.corners[1][0]});
            max_x = std::max({box.corners[0][0], box.corners[1][0]});
            min_y = std::min({box.corners[0][1], box.corners[1][1]});
            max_y = std::max({box.corners[0][1], box.corners[1][1]});

            if (point[0] > min_x && point[0] < max_x && point[1] > min_y && point[1] < max_y){
                is_grid_in_box = true;
            }
            break;
        default:
            break;
    }

    return is_grid_in_box;
}

/**
 * @brief 检查点是否在多边形内
 * 
 * @param  n：多边形顶点数量；point[2]：点；*vert[2]：多边形顶点
 * @return 0：点不在多边形内；1：点在多边形内
 */
int WeightedAreas::pnpoly(size_t n, const double point[2], const double (*verts)[2]){
    int i, j, c = 0;
	
	for (i = 0, j = n - 1; i < n; j = i++)
	{
        if (((verts[i][1] > point[1]) != (verts[j][1] > point[1])) &&
            (point[0] < (verts[j][0] - verts[i][0]) * (point[1] - verts[i][1]) / (verts[j][1] - verts[i][1]) + verts[i][0])) {
            c = !c;
        }
	}	
	return c;
}

}  // namespace amcl
}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

