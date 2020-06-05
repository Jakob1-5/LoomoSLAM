#include "LoomoCSVReader.h"



std::list<std::vector<std::string>> LoomoCSVReader::getData(std::string fileName) {
    clock_t tic = clock();
    std::ifstream file(fileName);

    if (!file.is_open())
        std::cout << "file would not open" << std::endl;


    std::list<std::vector<std::string>> dataList;

    std::string line;
    int nLines = 0;
    while (getline(file, line)) {
        nLines++;
        std::vector<std::string> vec;
        std::string str = "";
        for (auto i : line) {
            if (i == ',') {
                vec.push_back(str);
                str = "";
            }
            else if (i == '\"') {
                continue;
            }
            else {
                str += i;
            }
        }
        vec.push_back(str);
        dataList.push_back(vec);
    }
    file.close();
    std::cout << nLines << " lines" << std::endl;
    float elapsedTime = ((float)(clock() - tic)) / CLOCKS_PER_SEC;
    std::cout << "getData() took " << elapsedTime << " seconds" << std::endl;
    return dataList;
}


std::vector<CSVData> LoomoCSVReader::string2data(std::list<std::vector<std::string>> csvStrings) {
    std::vector<CSVData> data;
    for (const auto & line : csvStrings) {
        if (line == csvStrings.front())
            continue;
        data.push_back({
            stoll(line[dataColumn::TIME]),
            stoi(line[dataColumn::IR_LEFT]),
            stoi(line[dataColumn::IR_RIGHT]),
            stoi(line[dataColumn::ULTRASONIC]),
            stod(line[dataColumn::POSE_X]),
            stod(line[dataColumn::POSE_Y]),
            stod(line[dataColumn::POSE_THETA]),
            stod(line[dataColumn::POSE_LIN_VEL]),
            stod(line[dataColumn::POSE_ANG_VEL]),
            stoi(line[dataColumn::TICK_LEFT]),
            stoi(line[dataColumn::TICK_RIGHT]),
            stod(line[dataColumn::IMU_ROLL]),
            stod(line[dataColumn::IMU_PITCH]),
            stod(line[dataColumn::IMU_YAW]),
            stoll(line[dataColumn::FISHEYE_IDX]),
            stoll(line[dataColumn::COLOR_IDX]),
            stoll(line[dataColumn::DEPTH_IDX])
            });
    }
    return data;
}