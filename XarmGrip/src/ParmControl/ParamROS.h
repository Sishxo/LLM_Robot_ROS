#ifndef PARAMROS_H_
#define PARAMROS_H_

#include "common.h"

namespace XARMGRIP_CONTROLLER
{
    class ParamROS
    {
        public:
            ~ParamROS() {};
            ParamROS(const ParamROS&) = delete;
            ParamROS& operator=(const ParamROS&) = delete;
            static ParamROS& Instance() {
                static ParamROS m_pInstance;
                return m_pInstance;
        
            }
        private:
            ParamROS(){};

        public:
            Param GetParam();

            /// @brief 初始化
            void Init();
            ros::NodeHandle GetRosNode()
            {
                return Node_;
            }
        private:
            template<typename T> T getParam(const std::string& name,const T& defaultValue);
            ros::NodeHandle Node_;
            Param out;

    };
} 

#endif