#ifndef _RULERMVS_CORE_JSON_HPP_
#define _RULERMVS_CORE_JSON_HPP_
#include "rulermvs/core.hpp"
#include "rulermvs/pose.hpp"
#include "rulermvs/util.hpp"
#include "rulermvs/rect.hpp"
#include "rulermvs/camera.hpp"
#include "rulermvs/logger.hpp"
#include "rulermvs/matchinfo.hpp"
#include <fstream>
#include <json/json.h>
namespace rulermvs
{

template <typename T, typename = void> struct JsonSerialize {
    static inline void to(T& obj, Json::Value& val) {}
    static inline void from(Json::Value& val, T& obj) {}
};
template <typename T> void toJson(const T& obj, Json::Value& val)
{
    JsonSerialize<T>::to(obj, val);
}
template <typename T> void fromJson(const Json::Value& json, T& obj)
{
    JsonSerialize<T>::from(json, obj);
}

template <typename Tp> struct JsonSerialize<Point3_<Tp>,
    typename std::enable_if<!std::is_floating_point<Tp>::value>::type> {
    static inline void to(const Point3_<Tp>& obj, Json::Value& val)
    {
        val[0] = obj.x, val[1] = obj.y, val[2] = obj.z;
    }
    static inline void from(const Json::Value& val, Point3_<Tp>& obj)
    {
        obj.x = val[0].asInt(), obj.y = val[1].asInt(), obj.z = val[2].asInt();
    }
};
template <typename Tp> struct JsonSerialize<Point3_<Tp>,
    typename std::enable_if<std::is_floating_point<Tp>::value>::type> {
    static inline void to(const Point3_<Tp>& obj, Json::Value& val)
    {
        val[0] = (double)obj.x, val[1] = (double)obj.y, val[2] = (double)obj.z;
    }
    static inline void from(const Json::Value& val, Point3_<Tp>& obj)
    {
        obj.x = val[0].asDouble();
        obj.y = val[1].asDouble();
        obj.z = val[2].asDouble();
    }
};
template <typename Tp> struct JsonSerialize<Rect_<Tp>,
    typename std::enable_if<std::is_integral<Tp>::value>::type> {
    static inline void to(const Rect_<Tp>& obj, Json::Value& val)
    {
        val[0] = (int)obj.x, val[1] = (int)obj.y, val[2] = (int)obj.width,
        val[3] = (int)obj.height;
    }
    static inline void from(const Json::Value& val, Rect_<Tp>& obj)
    {
        obj.x      = val[0].asInt();
        obj.y      = val[1].asInt();
        obj.width  = val[2].asInt();
        obj.height = val[3].asInt();
    }
};
template <> struct JsonSerialize<Rotation> {
    static inline void to(const Rotation& obj, Json::Value& val)
    {
        coord_traits_t<Rotation> quat[4];
        toQuaternian(obj, quat);
        val["qw"] = (double)quat[0], val["qx"] = (double)quat[1];
        val["qy"] = (double)quat[2], val["qz"] = (double)quat[3];
    }
    static inline void from(const Json::Value& val, Rotation& obj)
    {
        coord_traits_t<Rotation> quat[4] = {0};

        quat[0] = val["qw"].asDouble();
        quat[1] = val["qx"].asDouble();
        quat[2] = val["qy"].asDouble();
        quat[3] = val["qz"].asDouble();
        fromQuaternian(quat, &obj.a1);
    }
};
template <> struct JsonSerialize<Pose> {
    static inline void to(const Pose& obj, Json::Value& val)
    {
        JsonSerialize<Point3_<coord_traits_t<Pose>>>::to(obj, val["Position"]);
        JsonSerialize<Rotation>::to(obj, val["Quaternion"]);
    }
    static inline void from(const Json::Value& val, Pose& obj)
    {
        JsonSerialize<Point3_<coord_traits_t<Pose>>>::from(
            val["Position"], obj);
        JsonSerialize<Rotation>::from(val["Quaternion"], obj);
    }
};
template <> struct JsonSerialize<std::vector<Pose>> {
    static inline void to(const std::vector<Pose>& obj, Json::Value& val)
    {
        auto& rts = val["RTs"];
        for (int i = 0; i < (int)obj.size(); ++i)
            JsonSerialize<Pose>::to(obj[i], rts[i]);
    }
    static inline void from(const Json::Value& val, std::vector<Pose>& obj)
    {
        auto& rts = val["RTs"];
        if (rts.empty()) return;
        obj.resize(rts.size());
        for (int i = 0; i < (int)rts.size(); ++i)
            JsonSerialize<Pose>::from(rts[i], obj[i]);
    }
};
template <> struct JsonSerialize<CameraP> {
    static inline void to(const CameraP& obj, Json::Value& val)
    {
        val["fx"]     = (double)obj.fx;
        val["fy"]     = (double)obj.fy;
        val["cx"]     = (double)obj.cx;
        val["cy"]     = (double)obj.cy;
        val["width"]  = obj.width;
        val["height"] = obj.height;
    }
    static inline void from(const Json::Value& val, CameraP& obj)
    {
        obj.fx     = val["fx"].asDouble();
        obj.fy     = val["fy"].asDouble();
        obj.cx     = val["cx"].asDouble();
        obj.cy     = val["cy"].asDouble();
        obj.width  = val["width"].asInt();
        obj.height = val["height"].asInt();
    }
};
template <> struct JsonSerialize<CameraSkewP> {
    static inline void to(const CameraSkewP& obj, Json::Value& val)
    {
        val["fx"]     = (double)obj.fx;
        val["fy"]     = (double)obj.fy;
        val["cx"]     = (double)obj.cx;
        val["cy"]     = (double)obj.cy;
        val["sk"]     = (double)obj.sk;
        val["width"]  = obj.width;
        val["height"] = obj.height;
    }
    static inline void from(const Json::Value& val, CameraSkewP& obj)
    {
        obj.fx     = val["fx"].asDouble();
        obj.fy     = val["fy"].asDouble();
        obj.cx     = val["cx"].asDouble();
        obj.cy     = val["cy"].asDouble();
        obj.sk     = val["sk"].asDouble();
        obj.width  = val["width"].asInt();
        obj.height = val["height"].asInt();
    }
};
template <> struct JsonSerialize<CameraPB> {
    static inline void to(const CameraPB& obj, Json::Value& val)
    {
        val["fx"]     = (double)obj.fx;
        val["fy"]     = (double)obj.fy;
        val["cx"]     = (double)obj.cx;
        val["cy"]     = (double)obj.cy;
        val["k1"]     = (double)obj.k1;
        val["k2"]     = (double)obj.k2;
        val["p1"]     = (double)obj.p1;
        val["p2"]     = (double)obj.p2;
        val["k3"]     = (double)obj.k3;
        val["width"]  = obj.width;
        val["height"] = obj.height;
    }
    static inline void from(const Json::Value& val, CameraPB& obj)
    {
        obj.fx     = val["fx"].asDouble();
        obj.fy     = val["fy"].asDouble();
        obj.cx     = val["cx"].asDouble();
        obj.cy     = val["cy"].asDouble();
        obj.k1     = val["k1"].asDouble();
        obj.k2     = val["k2"].asDouble();
        obj.p1     = val["p1"].asDouble();
        obj.p2     = val["p2"].asDouble();
        obj.k3     = val["k3"].asDouble();
        obj.width  = val["width"].asInt();
        obj.height = val["height"].asInt();
    }
};
template <> struct JsonSerialize<CameraSkewPB> {
    static inline void to(const CameraSkewPB& obj, Json::Value& val)
    {
        val["fx"]     = (double)obj.fx;
        val["fy"]     = (double)obj.fy;
        val["cx"]     = (double)obj.cx;
        val["cy"]     = (double)obj.cy;
        val["sk"]     = (double)obj.sk;
        val["k1"]     = (double)obj.k1;
        val["k2"]     = (double)obj.k2;
        val["p1"]     = (double)obj.p1;
        val["p2"]     = (double)obj.p2;
        val["k3"]     = (double)obj.k3;
        val["width"]  = obj.width;
        val["height"] = obj.height;
    }
    static inline void from(const Json::Value& val, CameraSkewPB& obj)
    {
        obj.fx     = val["fx"].asDouble();
        obj.fy     = val["fy"].asDouble();
        obj.cx     = val["cx"].asDouble();
        obj.cy     = val["cy"].asDouble();
        obj.sk     = val["sk"].asDouble();
        obj.k1     = val["k1"].asDouble();
        obj.k2     = val["k2"].asDouble();
        obj.p1     = val["p1"].asDouble();
        obj.p2     = val["p2"].asDouble();
        obj.k3     = val["k3"].asDouble();
        obj.width  = val["width"].asInt();
        obj.height = val["height"].asInt();
    }
};
template <> struct JsonSerialize<MatchInfo> {
    static inline void to(const MatchInfo& obj, Json::Value& val)
    {
        val["SrcID"] = obj.src_id;
        val["DstID"] = obj.dst_id;
        JsonSerialize<Pose>::to(obj.measure, val["Measure"]);
        auto& info = val["Information"];
        for (int i = 0; i < 36; ++i) info[i] = obj.info[i];
    }
    static inline void from(const Json::Value& val, MatchInfo& obj)
    {
        obj.src_id = val["SrcID"].asInt();
        obj.dst_id = val["DstID"].asInt();
        JsonSerialize<Pose>::from(val["Measure"], obj.measure);
        auto& info = val["Information"];
        if (info.size() == 36)
            for (int i = 0; i < 36; ++i) obj.info[i] = info[i].asDouble();
    }
};
template <> struct JsonSerialize<std::vector<MatchInfo>> {
    static inline void to(const std::vector<MatchInfo>& obj, Json::Value& val)
    {
        auto& infos = val["MatchInfos"];
        for (int i = 0; i < (int)obj.size(); ++i)
            JsonSerialize<MatchInfo>::to(obj[i], infos[i]);
    }
    static inline void from(const Json::Value& val, std::vector<MatchInfo>& obj)
    {
        auto& infos = val["MatchInfos"];
        if (infos.empty()) return;
        obj.resize(infos.size());
        for (int i = 0; i < (int)infos.size(); ++i)
            JsonSerialize<MatchInfo>::from(infos[i], obj[i]);
    }
};
template <typename T> bool readJson(ConstStr& path, T& obj)
{
    std::ifstream fin(path);
    if (!fin.is_open()) return false;
    Json::Value                             root;
    JSONCPP_STRING                          doc, errs;
    Json::CharReaderBuilder                 reader;
    std::unique_ptr<Json::CharReader> const jsonReader(reader.newCharReader());
    std::getline(fin, doc, (char)EOF);
    if (!jsonReader->parse(doc.data(), doc.data() + doc.size(), &root, &errs)) {
        MVS_ILOG << "Error: json parse failed.";
        return false;
    }
    JsonSerialize<T>::from(root, obj);
    fin.close();
    return true;
}
template <typename T> bool writeJson(ConstStr& path, const T& obj)
{
    std::ofstream fout;
    fout.open(path.c_str());
    if (!fout.is_open()) return false;
    Json::Value               root;
    Json::StreamWriterBuilder writerBuilder;
    writerBuilder["commentStyle"] = "None";
    // writerBuilder["dropNullPlaceholders"] = true;
    writerBuilder["indentation"] = "";  // or whatever you like
    std::unique_ptr<Json::StreamWriter> jsonWriter(
        writerBuilder.newStreamWriter());
    JsonSerialize<T>::to(obj, root);
    jsonWriter->write(root, &fout);
    fout.close();
    return true;
}

}  // namespace rulermvs
#endif