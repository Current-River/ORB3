/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

/**
 * @brief 获取图像文件的信息
 * @param[in]  strImagePath     图像文件存放路径
 * @param[in]  strPathTimes     时间戳文件的存放路径
 * @param[out] vstrImages       图像文件名数组
 * @param[out] vTimeStamps      时间戳数组
 */
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;

/*
* int 型argc      统计运行时发送给main函数的命令行参的个数，VS中默认为1，该处传入至少为5个
*                 可执行文件、ORB词典文件、Monocular-IMU配置文件、数据集文件、时间戳.txt配置文件
* char* 型argv[]  字符串数组：存放指向字符串参数的指针数组
*                 argv[0] 指向程序运行的全路径名
*                 argv[1] 指向在DOS命令中执行程序名后的第一个字符串
*                 argv[2] 指向执行程序名后的第二个字符串
*                 argv[3] 指向执行程序名后的第三个字符串
*                   ...
*                 argv[argc]为NULL (6)  
*/

int main(int argc, char *argv[])
{
    // step 0 检查输入参数是否足够 
    if(argc < 5)
    {
        // cerr 输出到标准错误ostream对象(不进入缓存)，用于程序错误信息，这里是传入参数至少为5个
        // return 1 代表异常退出
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }

    // step 1 多组数据时，进行逐个执行
    // 序列数目？此时为1对序列（减去前三个配置文件，后边每两对为一组数据）
    // 每对中，第一个为数据文件，第二个为数据对应时间戳文件
    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // step 2 加载图像和IMU的值
    // Load all sequences:
    // 图像序列的文件名字符串序列
    // 相机时间戳
    // 加速度计、陀螺仪3D坐标
    // 当前图像序列中图像的数目、IMU数目
    // 第一帧imu赋值0？
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    // num_seq个序列的容量
    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;
        // cout << "run SLAM? " << endl;

        nImages[seq] = vstrImageFilenames[seq].size();      // 每个序列中图像的个数
        tot_images += nImages[seq];                         // 下一张图像？
        nImu[seq] = vTimestampsImu[seq].size();             // 每个序列中IMU的个数

        // 是否还有IMU和图像的数值判断
        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        // 将第一个IMU的值作为起始值？
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }

    // Vector for tracking time statistics
    // 统计追踪一帧耗时 (仅Tracker线程)
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    // 输出流控制函数，返回当前浮点数的精度值：保留小数点后的16位
    cout.precision(17);

    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/

    cout<<"----------加载SLAM系统的分界线，这里进入system----------"<<endl;
    // step 3 加载SLAM系统
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],                             // path_to_vocabulary
                           argv[2],                             // path_to_settings
                           ORB_SLAM3::System::IMU_MONOCULAR,    // 单目-imu模式
                           true);                               // 启用可视化查看器

    cout<<"----------结束SLAM系统前期加载，进入正式数据处理模式的分界线----------"<<endl;
    // step 4 遍历每一组序列
    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        // step 5 追踪序列中的每一张图像、IMU数据
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            // step 5.1 读根据前面获得的图像文件名读取图像,读取过程中不改变图像的格式        
            im = cv::imread(vstrImageFilenames[seq][ni],CV_LOAD_IMAGE_UNCHANGED);

            double tframe = vTimestampsCam[seq][ni];

            // step 5.2 图像的合法性检查
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
            {
                // cout << "t_cam " << tframe << endl;

                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }

            /*cout << "first imu: " << first_imu << endl;
            cout << "first imu time: " << fixed << vTimestampsImu[first_imu] << endl;
            cout << "size vImu: " << vImuMeas.size() << endl;*/

            // step 5.3 开始计时
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            cout << "tframe = " << tframe << endl;
            // step 5.4 追踪当前图像，IMU
            SLAM.TrackMonocular(im,tframe,vImuMeas); // TODO change to monocular_inertial

            // step 5.5 追踪完成，停止当前帧的图像计时，并计算耗时
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;
            std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            // step 5.6 根据图像时间戳中记录的两张图像之间的时间和现在追踪当前图像所耗费的时间,继续等待指定的时间以使得下一张图像能够
            // 按照时间戳被送入到SLAM系统中进行跟踪        
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }

        // step 6 若存在多组序列，则运行完一组后，加载另一组
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    // step 7 若所有图像都预测完成，终止当前SLAM系统
    SLAM.Shutdown();

    // Save camera trajectory
    // step 8 ORB2中保存的是TUM格式的轨迹
    // 猜测是单目时有尺度漂移, 而LGA GBA都只能优化关键帧使尺度漂移最小, 普通帧所产生的轨迹漂移这里无能为力, 我猜作者这样就只
    // 保存了关键帧的位姿,从而避免普通帧带有尺度漂移的位姿对最终误差计算的影响
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    // 打开文件
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    // 遍历文件
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        // 只有当前行不为空的时候执行
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            // 生成当前行所指出的RGB图像的文件名称
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            // 记录该图像的时间戳
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        // 每行数据存入data[]中，每个data[]压入栈中
        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
