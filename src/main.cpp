#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <limits>
#include <json/json.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>

int load_json(const std::string& path, Json::Value * out_value, Json::String * errs) {
    std::ifstream ifs;
    ifs.open(path);
    Json::CharReaderBuilder rBuilder;
    rBuilder["collectComments"] = false;
    if (!parseFromStream(rBuilder, ifs, out_value, errs)) {
        return false;
    }
    return true;
}

class GeoMap {
public:
    typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> point;
    typedef boost::geometry::model::box<point> box;
    typedef std::pair<box, std::pair<Json::Value, Json::Value>> value;

    boost::geometry::index::rtree<value, boost::geometry::index::quadratic<16>> rtree;

    int build(Json::Value& geo_maps) {
        if (geo_maps["features"].isNull()) return false;

        Json::Value& features = geo_maps["features"];
        if (!features.isArray()) return false;

        size_t total = features.size();
        for (size_t i = 0; i < total; ++i) {
            Json::Value& feature = features[(Json::ArrayIndex)i];
            if (!feature["geometry"].isObject()) continue;

            Json::Value& geometry = feature["geometry"];
            if (!geometry["coordinates"].isArray()) continue;
            if (!geometry["type"].isString()) continue;

            Json::String type = geometry["type"].asString();
            Json::Value& coordinates = geometry["coordinates"];
            Json::Value& properties = feature["properties"];
            if (type == "Polygon") {
                this->add_polygon(coordinates, properties);
            } else if (type == "MultiPolygon") {
                size_t total_polygon = coordinates.size();
                for (size_t polygon_index = 0; polygon_index < total_polygon; ++polygon_index) {
                    this->add_polygon(coordinates[(Json::ArrayIndex)polygon_index], properties);
                }
            }
        }
        return true;
    }

    void query(float longitude, float latitude, std::vector<std::pair<Json::Value, Json::Value>>& out_query, bool multi = false) {
        std::vector<value> result_n;
        box query_box(point(longitude, latitude), point(longitude, latitude));
        this->rtree.query(boost::geometry::index::intersects(query_box), std::back_inserter(result_n));
        bool first = true;
        BOOST_FOREACH(value const& v, result_n)
            if (inside_polygon(v.second.first, longitude, latitude)) {
                if (multi || first) {
                    out_query.push_back(v.second);
                }
                first = false;
            }
    }

private:
    void add_polygon(Json::Value& polygon, Json::Value& properties) {
        const float POS_INF = std::numeric_limits<float>::infinity();
        const float NEG_INF = -std::numeric_limits<float>::infinity();
        float minX = POS_INF, minY = POS_INF, maxX = NEG_INF, maxY = NEG_INF;
        Json::Value& polygon_0 = polygon[0];
        size_t num_points = polygon_0.size();
        for (size_t point_index = 0; point_index < num_points; point_index++){
            Json::Value& p = polygon_0[(Json::ArrayIndex)point_index];
            minX = std::min(minX, p[0].asFloat());
            minY = std::min(minY, p[1].asFloat());
            maxX = std::max(maxX, p[0].asFloat());
            maxY = std::max(maxY, p[1].asFloat());
        }
        box b(point(minX, minY), point(maxX, maxY));
        this->rtree.insert(std::make_pair(b, std::make_pair(polygon, properties)));
    }

    bool inside_polygon(const Json::Value& rings, float centre_x, float centre_y) {
        bool inside = false;
        for (size_t i = 0, len = rings.size(); i < len; i++) {
            const Json::Value& ring = rings[(Json::ArrayIndex)i];
            for (size_t j = 0, len2 = ring.size(), k = len2 - 1; j < len2; k = j++) {
                auto p1x = ring[(int)j][0].asFloat();
                auto p1y = ring[(int)j][1].asFloat();
                auto p2x = ring[(int)k][0].asFloat();
                auto p2y = ring[(int)k][1].asFloat();
                if (ray_intersect(centre_x, centre_y, p1x, p1y, p2x, p2y)) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    bool ray_intersect(float px, float py, float p1x, float p1y, float p2x, float p2y) {
        return ((p1y > py) != (p2y > py)) && (px < (p2x - p1x) * (py - p1y) / (p2y - p1y) + p1x);
    }
};

int main(int argc, const char * argv[]) {
    int verbose = 0;
    static struct option long_options[] =
    {
        {"border-json",     required_argument,  0, 'b'},
        {"annotation-json", required_argument,  0, 'a'},
        {"output",          required_argument,  0, 'o'},
        {"verbose",         no_argument,        &verbose, 1},
        {0, 0, 0, 0}
    };

    std::string border_json_file, annotation_json_file, output;

    int c;
    while (1)
    {
        int option_index = 0;
        c = getopt_long(argc, (char **)argv, "b:a:o:", long_options, &option_index);

        if (c == -1) {
            break;
        }

        switch (c)
        {
            case 'b':
                border_json_file = optarg;
                break;
            case 'a':
                annotation_json_file = optarg;
                break;
            case 'o':
                output = optarg;
                break;
            default:
                break;
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    Json::Value annotation_root, geo_maps_root;
    JSONCPP_STRING errs;
    if (verbose) printf("[+] Loading border JSON file from %s...\r\n", border_json_file.c_str());
    if (!load_json(border_json_file, &geo_maps_root, &errs)) {
        std::cout << "[!] " << errs << '\n';
        return EXIT_FAILURE;
    }

    GeoMap geo_map;
    if (verbose) printf("[+] Building R-Tree...\r\n");
    if (!geo_map.build(geo_maps_root)) {
        std::cout << "[!] " <<  "Failed to build geo maps\n";
        return EXIT_FAILURE;
    }

    std::vector<std::pair<Json::Value, Json::Value>> result;

    if (verbose) printf("[+] Loading annotation JSON file from %s...\r\n", annotation_json_file.c_str());
    if (!load_json(annotation_json_file, &annotation_root, &errs)) {
        std::cout << "[!] " << errs << std::endl;
        return EXIT_FAILURE;
    }

    Json::Value& images = annotation_root["images"];
    size_t total = 0, valid = 0, failed = 0;
    if (images.isArray()) {
        total = images.size();
        if (verbose) printf("[+] Querying for %lu images...\r\n", total);
        for (size_t index = 0; index < total; ++index) {
            if (verbose) {
                printf("\r[-] Current querying for image %lu...", index);
                fflush(stdout);
            }
            Json::Value& image = images[(Json::ArrayIndex)index];
            if (image["longitude"].isNumeric() && image["latitude"].isNumeric()) {
                float longitude = image["longitude"].asFloat();
                float latitude = image["latitude"].asFloat();
                geo_map.query(longitude, latitude, result);
                if (result.size() == 1 && result[0].second["country"].isString()) {
                    image["country"] = result[0].second["country"].asString();
                } else {
                    failed++;
                }
                result.clear();
                valid++;
            }
        }
        if (verbose) printf("\r\n");
    } else {
        if (verbose) printf("[!] Invalid annotation JSON\r\n");
        return EXIT_FAILURE;
    }

    if (verbose) printf("[+] Writing queried results to %s...\r\n", output.c_str());
    Json::StreamWriterBuilder wBuilder;
    wBuilder["commentStyle"] = "None";
    wBuilder["indentation"] = "";
    std::unique_ptr<Json::StreamWriter> writer(wBuilder.newStreamWriter());
    std::ofstream outputFileStream(output);
    writer->write(annotation_root, &outputFileStream);

    auto end = std::chrono::high_resolution_clock::now();
    if (verbose) printf("[+] Processing time: %lfs\n", ((std::chrono::duration<double>)((end - start))).count());
    if (verbose) printf("[+] Total=%lu, valid=%lu, failed=%lu\n", total, valid, failed);
}
