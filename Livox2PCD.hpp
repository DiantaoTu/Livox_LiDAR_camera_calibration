/*
 * @Author: your name
 * @Date: 2021-09-24 13:16:51
 * @LastEditTime: 2021-09-29 17:21:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /livox_lidar_camera_calib/Livox2PCD.hpp
 */

#include "livox_def.h"
#include "livox_file.h"
#include <string>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "base.hpp"

using namespace std;

// 把时间戳转化为字符串,并且要用两位数字表示,例如 7->07  3->03
string ConvertTimeStampToStr(uint8_t *timestamp)
{
    int year = timestamp[0];
    int month = timestamp[1];
    int day = timestamp[2];
    int hour = timestamp[3];
    uint32_t microseconds = timestamp[4] | timestamp[5] << 8 | timestamp[6] << 16 | timestamp[7] << 24;
    // cout << year << " " << month << " " << day << " " << hour << " " << microseconds << endl;
    float seconds = microseconds / 1000000.f;
    int minute = seconds / 60;
    seconds = seconds - minute * 60;
    int ms = (seconds - int(seconds)) * 1000;
    char s[100];
    // 年月日_时分秒  210126_071205788
    sprintf(s, "%2d%2d%2d_%2d%2d%2d%3d", year, month, day, hour, minute, int(seconds), ms);
    string str(s);
    int pos = str.find(' ');
    while (pos != -1)
    {
        str.replace(pos, 1, "0");
        pos = str.find(' ');
    }
    return str;
}

double ConvertTimeStampToDouble(uint8_t *timestamp, int type = 0)
{
    if(type == kTimestampTypeNoSync)
    {
        uint64_t n_sec = 0;
        for(int i = 7; i >= 0; i--)
            n_sec = n_sec << 8 | timestamp[i];
        double second = n_sec;
        second /= 1e9;
        return second;
    }
    else 
    {
        int year = timestamp[0];
        int month = timestamp[1];
        int day = timestamp[2];
        int hour = timestamp[3];
        uint32_t microseconds = timestamp[4] | timestamp[5] << 8 | timestamp[6] << 16 | timestamp[7] << 24;
        double seconds = microseconds / 1000000.f;
        seconds += hour * 3600.f;

        return seconds;
    }
}

int AccumulateCloud(vector<LvxBasePackDetail> &pcl_raw, string name)
{
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    int idx = 0;
    for (idx = 0; idx < pcl_raw.size(); idx++)
    {
        LvxBasePackDetail packet = pcl_raw[idx];
        for (int i = 0; i < SINGLE_POINT_NUM; i++)
        {
            LivoxExtendRawPoint point;
            memcpy(&point, packet.raw_point + i * sizeof(LivoxExtendRawPoint), sizeof(LivoxExtendRawPoint));
            // if (point.x == 0 || point.y == 0 || point.z == 0)
            //     continue;
            // 根据tag判断当前点是否为噪点，如果是噪点，就不计算了。tag的详细说明见
            // https://livox-wiki-cn.readthedocs.io/zh_CN/latest/introduction/Point_Cloud_Characteristics_and_Coordinate_System%20.html
            if((point.tag & 0x0c) == 4)     // 0x0c = 00001100
                continue;
            if((point.tag & 0x03) == 1)     // 0x03 = 00000011
                continue;

            pcl::PointXYZI pt;
            pt.x = point.x / 1000.f;
            pt.y = point.y / 1000.f;
            pt.z = point.z / 1000.f;
            pt.intensity = point.reflectivity;
            point_cloud.push_back(pt);
        }
    }
    pcl_raw.erase(pcl_raw.begin(), pcl_raw.begin() + idx);
    pcl::io::savePCDFileASCII(name, point_cloud);
    return 1;
}

// 读取初始的部分 PublicHeader + PrivateHeader + DeviceInfo
int ReadHead(ifstream &lvxFile)
{
    cout << "------------read lvx file header begin-------------" << endl;
    // 先读取24字节的LvxFilePublicHeader
    LvxFilePublicHeader public_header;
    lvxFile.read((char *)&public_header, sizeof(LvxFilePublicHeader));
    cout << "signature: " << public_header.signature << endl;
    cout << "version: ";
    for (int i = 0; i < sizeof(public_header.version); i++)
        cout << (int)public_header.version[i] << " ";
    cout << endl;

    // 再读取5字节的LvxFilePrivateHeader
    LvxFilePrivateHeader private_header;
    lvxFile.read((char *)&private_header, sizeof(LvxFilePrivateHeader));
    // cout << sizeof(LvxFilePrivateHeader) << endl;
    cout << "frame duration: " << private_header.frame_duration << endl;
    cout << "device count: " << (int)private_header.device_count << endl;

    LvxDeviceInfo device_info;      // 59 Byte
    for (int i = 0; i < private_header.device_count; i++)
        lvxFile.read((char *)&device_info, sizeof(device_info));

    cout << "------------read lvx file header end-------------" << endl;

    return 0;
}

// 读取数据部分 数据部分的格式为 1个FrameHeader + n个packet
int ReadData(ifstream &lvxFile, double pcd_duration, string lvxName)
{
    cout << "------------read lvx file data begin-------------" << endl;
    string base_name = lvxName.substr(0, lvxName.size() - 4);
    std::vector<LvxBasePackDetail> pcl_raw;
    int frame_count = 0, packet_count = 0;
    int pcd_count = 0;
    double start_time = -1, curr_time = -1;

    uint64_t curr_offset = 0; // 当前文件流的偏移量
    uint64_t next_offset = 0; // 下一个FrameHeader的偏移量
    uint64_t length = 0;
    streampos pos;
    pos = lvxFile.tellg();
    lvxFile.seekg(0, ios::end);
    length = (uint64_t)lvxFile.tellg() / 1024 / 1024;
    lvxFile.seekg(pos, ios::beg);

    while (!lvxFile.eof())
    {
        // 读取 FrameHeader
        FrameHeader frame_header;
        lvxFile.read((char *)&frame_header, sizeof(FrameHeader));
        // cout << "frame index: " << frame_header.frame_index << endl;
        // cout << "current offset: " << frame_header.current_offset << endl;
        // cout << "next offset: " << frame_header.next_offset << endl;
        curr_offset = frame_header.current_offset;
        next_offset = frame_header.next_offset;
        curr_offset += sizeof(FrameHeader);     // 读完一个framheader后，curr_offset应该增加
        pos = lvxFile.tellg();
        if (pos == -1)
        {
            break;
        }
        assert(curr_offset == (uint64_t)pos);
        // cout << "processing :" << curr_offset / 1024 / 1024 << "MB / " << length << "MB" << endl;

        // 读取连续的n个packet
        while (curr_offset < next_offset)
        {
            // 先向后移动10个字节，得到data type
            lvxFile.seekg(10, ios::cur);
            uint8_t data_type = lvxFile.get();
            // cout << "data type: " << (int)data_type << endl;
            // 再向前移动11个字节，回到原位置
            lvxFile.seekg(-11, ios::cur);

            LvxBasePackDetail packet;
            packet.data_type = data_type;

            if (packet.data_type != PointDataType::kExtendCartesian)
                cout << "data type is :" << packet.data_type << endl;

            // 不同类型的data会有不同的长度
            switch (packet.data_type)
            {
            case PointDataType::kCartesian:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + RAW_POINT_NUM * sizeof(LivoxRawPoint);
                break;
            case PointDataType::kSpherical:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + RAW_POINT_NUM * sizeof(LivoxSpherPoint);
                break;
            case PointDataType::kExtendCartesian:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + SINGLE_POINT_NUM * sizeof(LivoxExtendRawPoint);
                break;
            case PointDataType::kExtendSpherical:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + SINGLE_POINT_NUM * sizeof(LivoxExtendSpherPoint);
                break;
            case PointDataType::kDualExtendCartesian:
                cout << int(PointDataType::kDualExtendCartesian) << endl;
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + DUAL_POINT_NUM * sizeof(LivoxDualExtendRawPoint);
                break;
            case PointDataType::kDualExtendSpherical:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + DUAL_POINT_NUM * sizeof(LivoxDualExtendSpherPoint);
                break;
            case PointDataType::kImu:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + IMU_POINT_NUM * sizeof(LivoxImuPoint);
                break;
            case PointDataType::kTripleExtendCartesian:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendRawPoint);
                break;
            case PointDataType::kTripleExtendSpherical:
                packet.pack_size = sizeof(LvxBasePackDetail) - sizeof(packet.raw_point) - sizeof(packet.pack_size) + TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendSpherPoint);
                break;
            default:
                cout << "data type error" << endl;
                break;
            }
            // 读取当前packet，长度就是刚才计算得到的
            lvxFile.read((char *)&packet, packet.pack_size);
            // cout << "packet size: " << packet.pack_size << endl;
            curr_offset += packet.pack_size;
            pos = lvxFile.tellg();
            assert(curr_offset == (uint64_t)pos);

            if ((packet.data_type == PointDataType::kExtendCartesian))
            {
                pcl_raw.push_back(packet);
            }
            packet_count++;
            curr_time = ConvertTimeStampToDouble(packet.timestamp, packet.timestamp_type);
            if(start_time < 0)
                start_time = curr_time;
            else if( (pcd_duration > 0) && (curr_time - start_time >= pcd_duration))
            {
                pcd_count++;
                string name = int2str(pcd_count);
                name = base_name + "_" + name + ".pcd";
                AccumulateCloud(pcl_raw, name);
                start_time = -1;        // reset start time
            }
        }
        frame_count++;
    }

    if (!pcl_raw.empty())
    {
        pcd_count++;
        // 如果只保存了一个pcd文件，那就不用在后面加 _1 _2 这种后缀了
        string name = int2str(pcd_count);
        name = base_name + "_" + name + ".pcd";
        if(pcd_count == 1)
            name = base_name + ".pcd";
        AccumulateCloud(pcl_raw, name);
    }
    cout << "frame count: " << frame_count << endl;
    cout << "packet count: " << packet_count << endl;
    cout << "------------read lvx file data end-------------" << endl;
    return 0;
}


/**
 * @description: 
 * @param {string} lvx_file_path, path to the folder contain all lvx data
 * @param {double} pcd_duration
 * @return {*}
 */
int Livox2PCD(std::string lvx_file_path, double pcd_duration)
{
    vector<string> lvx_file_names;
    IterateFiles(lvx_file_path, lvx_file_names, ".lvx");
    for (string name : lvx_file_names)
    {
        ifstream lvxFile;
        lvxFile.open(name, ios::binary | ios::in);
        if (!lvxFile.is_open())
        {
            cout << "can't open " << name << endl;
            return 0;
        }
        else
            cout << "converting " << name << " to pcd " << endl;
        
        ReadHead(lvxFile);
        ReadData(lvxFile, pcd_duration, name);

        lvxFile.close();
    }
    return 1;
}
