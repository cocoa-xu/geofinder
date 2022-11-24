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
#include <boost/asio.hpp>
#include <boost/thread.hpp>
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
    class Point {
    public:
        Point(float _x, float _y) : x(_x), y(_y) {}
        float x;
        float y;
    };
    
    class Ring {
    public:
        static Ring from_json_value(const Json::Value& ring) {
            Ring r;
            size_t num_points = ring.size();
            for (size_t point_index = 0; point_index < num_points; point_index++){
                const Json::Value& p = ring[(Json::ArrayIndex)point_index];
                r.points.emplace_back(p[0].asFloat(), p[1].asFloat());
            }
            return r;
        }

        Point& operator[](size_t index) {
            if (index > points.size()) {
                throw std::invalid_argument("index out of range");
            } else {
                return points[index];
            }
        }

        const Point& operator[](size_t index) const {
            if (index > points.size()) {
                throw std::invalid_argument("index out of range");
            } else {
                return points[index];
            }
        }

        size_t size() const {
            return points.size();
        }

        std::vector<Point> points;
    };

    class Polygon {
    public:
        static Polygon polygon_from_json(const Json::Value& polygon) {
            Polygon p;
            size_t num_rings = polygon.size();
            for (size_t ring_index = 0; ring_index < num_rings; ring_index++){
                const Json::Value& ring = polygon[(Json::ArrayIndex)ring_index];
                Ring r = Ring::from_json_value(ring);
                p.rings.emplace_back(r);
            }
            return p;
        }

        Ring& operator[](size_t index) {
            if (index > rings.size()) {
                throw std::invalid_argument("index out of range");
            } else {
                return rings[index];
            }
        }

        const Ring& operator[](size_t index) const {
            if (index > rings.size()) {
                throw std::invalid_argument("index out of range");
            } else {
                return rings[index];
            }
        }

        size_t size() const {
            return rings.size();
        }

        std::vector<Ring> rings;
    };

    typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> point;
    typedef boost::geometry::model::box<point> box;
    typedef std::pair<Polygon, Json::Value> polygon_with_property_t;
    typedef std::pair<box, polygon_with_property_t> value;
    typedef std::vector<polygon_with_property_t> query_t;

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

    void query(float longitude, float latitude, std::vector<std::pair<Polygon, Json::Value>>& out_query, bool multi = false) {
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

        Polygon p = Polygon::polygon_from_json(polygon);
        Ring& ring_0 = p[0];

        size_t num_points = ring_0.size();
        for (size_t point_index = 0; point_index < num_points; point_index++){
            Point& p = ring_0[point_index];
            minX = std::min(minX, p.x);
            minY = std::min(minY, p.y);
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
        }
        box b(point(minX, minY), point(maxX, maxY));
        this->rtree.insert(std::make_pair(b, std::make_pair(p, properties)));
    }

    bool inside_polygon(const Polygon& polygon, float centre_x, float centre_y) {
        bool inside = false;
        for (size_t i = 0, len = polygon.size(); i < len; i++) {
            const Ring& ring = polygon[i];
            for (size_t j = 0, len2 = ring.size(), k = len2 - 1; j < len2; k = j++) {
                if (ray_intersect(centre_x, centre_y, ring[j], ring[k])) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    bool ray_intersect(float px, float py, const Point& p1, const Point& p2) {
        return ((p1.y > py) != (p2.y > py)) && (px < (p2.x - p1.x) * (py - p1.y) / (p2.y - p1.y) + p1.x);
    }
};

int main(int argc, const char * argv[]) {
    int verbose = 0;
    int jobs = 0;
    static struct option long_options[] =
    {
        {"border-json",     required_argument,  0, 'b'},
        {"annotation-json", required_argument,  0, 'a'},
        {"jobs",            optional_argument,  0, 'j'},
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
            case 'j':
                jobs = atoi(optarg);
                break;
            default:
                break;
        }
    }

    if (jobs <= 0) {
        jobs = std::thread::hardware_concurrency();
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

    if (verbose) printf("[+] Loading annotation JSON file from %s...\r\n", annotation_json_file.c_str());
    if (!load_json(annotation_json_file, &annotation_root, &errs)) {
        std::cout << "[!] " << errs << std::endl;
        return EXIT_FAILURE;
    }

    Json::Value& images = annotation_root["images"];
    size_t total = 0, valid = 0;
    if (images.isArray()) {
        total = images.size();
        boost::asio::thread_pool pool(jobs);
        if (verbose) printf("[+] Querying for %lu images...\r\n", total);
        for (size_t index = 0; index < total; ++index) {
            Json::Value& image = images[(Json::ArrayIndex)index];
            if (image["longitude"].isNumeric() && image["latitude"].isNumeric()) {
                boost::asio::post(pool, [&image, &geo_map]() {
                    float longitude = image["longitude"].asFloat();
                    float latitude = image["latitude"].asFloat();
                    GeoMap::query_t result;
                    geo_map.query(longitude, latitude, result);
                    if (result.size() == 1 && result[0].second["country"].isString()) {
                        image["country"] = result[0].second["country"].asString();
                    }
                });
                valid++;
            }
        }
        pool.join();
    } else {
        printf("[!] Invalid annotation JSON\r\n");
        return EXIT_FAILURE;
    }

    if (verbose) printf("[+] Writing queried results to %s...\r\n", output.c_str());
    Json::StreamWriterBuilder wBuilder;
    wBuilder["commentStyle"] = "None";
    wBuilder["indentation"] = "";
    wBuilder["precision"] = 6;
    std::unique_ptr<Json::StreamWriter> writer(wBuilder.newStreamWriter());
    std::ofstream outputFileStream(output);
    writer->write(annotation_root, &outputFileStream);

    auto end = std::chrono::high_resolution_clock::now();
    if (verbose) printf("[+] Processing time: %lfs\n", ((std::chrono::duration<double>)((end - start))).count());
    if (verbose) printf("[+] Total=%lu, valid=%lu\n", total, valid);
}
