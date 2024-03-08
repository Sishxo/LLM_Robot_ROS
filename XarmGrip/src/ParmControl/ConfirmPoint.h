#ifndef CONFIRMPOINT_H_
#define CONFIRMPOINT_H_

#include "common.h"
#include "ConfirmYaml.h"
#include "commonType/TargetReco.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

namespace XARMGRIP_CONTROLLER
{
    class ConfirmPoint
    {
        public:
            ~ConfirmPoint() {};
            ConfirmPoint(const ConfirmPoint&) = delete;
            ConfirmPoint& operator=(const ConfirmPoint&) = delete;
            static ConfirmPoint& Instance() {
                static ConfirmPoint m_pInstance;
                return m_pInstance;
            }
        private:
            ConfirmPoint(){};

        public:
            /// @brief 初始化
            void Init();
            /// @brief 关闭
            void Close();
            /// @brief 获取yaml中点的值
            /// @param label 
            /// @param Points 
            void GetYamlPoints(std::string label, std::vector<commonType::TargetReco> &Points);
            /// @brief 更新yaml中点的值
            /// @param label 
            /// @param id 
            /// @param points 
            void SetYamlPoint(std::string label, int id, commonType::TargetReco points);
            /// @brief 获取label个数
            /// @param label 
            /// @param size 
            int GetYamlPointSize(std::string label);
            /// @brief 清除所有点
            void ClearYamlPoint(std::string label);
            void test();

        private:
            YAML::Node config_;
    };
} 

#endif