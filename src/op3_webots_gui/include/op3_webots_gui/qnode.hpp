/**
 * @file /include/op3_webots_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef op3_webots_gui_QNODE_HPP_
#define op3_webots_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>

#include "sensor_msgs/JointState.h"

#include <yaml-cpp/yaml.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>




/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_webots_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool parseJointNameFromYaml(const std::string &path);
    void sendJointValue(int joint_index, double joint_value);
    int getJointSize();
    std::string getJointNameFromIndex(int joint_index);

    void run();

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
    std::map<int, std::string> joint_names_;
    bool debug_;

    ros::Publisher desired_joint_state_pub_;

};

}  // namespace op3_webots_gui

#endif /* op3_webots_gui_QNODE_HPP_ */
