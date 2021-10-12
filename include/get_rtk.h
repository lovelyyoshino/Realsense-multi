//
// Created by jiao on 2021/5/31.
//

#ifndef GET_RGBD_FRAMES_GET_RTK_H
#define GET_RGBD_FRAMES_GET_RTK_H

#define READ_BUFFER 500

#include <vector>
#include <string>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

class BaseController{
public:
    BaseController(std::string serial_dev);
    void ReadData(std::vector<double> &rtk_datas);

private:
    std::string serial_dev_;
    int serial_fd_;
    void CombineRecv(const std::vector<unsigned char>& recv_raw, std::vector<unsigned char>& recv_combine);
    bool CompareCheckCode(const std::vector<unsigned char>& recv_combine);
    unsigned short CRC_ADD(const std::vector<unsigned char>& source_data, int offset_start, int offset_end);
    void SerialClose();
    int SerialOpen(std::string serial_dev, int baud_rate, int data_bits, char parity, int stop_bits, int vmin);
};

#endif //GET_RGBD_FRAMES_GET_RTK_H
