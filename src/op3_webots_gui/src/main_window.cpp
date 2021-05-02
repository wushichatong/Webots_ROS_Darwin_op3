/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/op3_webots_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_webots_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
  setWindowTitle("Webots Robotis-OP3 Joint Contorller");
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  initJointSliders();

}

MainWindow::~MainWindow() {}




void MainWindow::initJointSliders(){
  // joints
  QGridLayout *grid_layout = new QGridLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);

  for (int ix = 0; ix < qnode.getJointSize(); ix++)
  {
    std::stringstream label_stream;
    std::string joint_name = qnode.getJointNameFromIndex(ix + 1);
//    std::string joint_name = ix + " ";
    QLabel *id_label = new QLabel(tr(joint_name.c_str()));


    QSlider *slider = new QSlider(this);
    slider->setOrientation(Qt::Horizontal);  // 水平方向
    slider->setMinimum(-50);  // 最小值
    slider->setMaximum(50);  // 最大值
    slider->setValue(0);
    slider->setSingleStep(1);  // 步长
    slider->setObjectName(QString("slider %1").arg(ix + 1));
    // 连接信号槽（相互改变）
//    connect(pSpinBox, SIGNAL(valueChanged(int)), pSlider, SLOT(setValue(int)));
    signalMapper->setMapping(slider, slider);
    QObject::connect(slider, SIGNAL(valueChanged(int)), signalMapper, SLOT(map()));
    int num_row = ix / 2 + 1;
    int num_col = (ix % 2) * 3;
    grid_layout->addWidget(id_label, num_row, num_col, 1, 1);
    grid_layout->addWidget(slider, num_row, num_col + 1, 1, 2);
  }
  QObject::connect(signalMapper, SIGNAL(mapped(QWidget *)), this, SLOT(setJointAngle(QWidget *)));
  ui.widget->setLayout(grid_layout);
}


void MainWindow::setJointAngle(QWidget *widget)
{
  QSlider *slider = qobject_cast<QSlider *>(widget);
  slider->value();
  QStringList list = slider->objectName().split(" ");
  if(list.size() >= 2){
    int joint_index = list[1].toInt();
    qnode.sendJointValue(joint_index, slider->value());
    ROS_INFO("send joint [%d] value: %d", joint_index, slider->value());
  }

}
/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_connect_clicked(bool check ) {
//  if ( ui.checkbox_use_environment->isChecked() ) {
//    if ( !qnode.init() ) {
//      showNoMasterMessage();
//    } else {
//      ui.button_connect->setEnabled(false);
//    }
//  } else {
//    if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
//           ui.line_edit_host->text().toStdString()) ) {
//      showNoMasterMessage();
//    } else {
//      ui.button_connect->setEnabled(false);
//      ui.line_edit_master->setReadOnly(true);
//      ui.line_edit_host->setReadOnly(true);
//      ui.line_edit_topic->setReadOnly(true);
//    }
//  }
//}


//void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
//  bool enabled;
//  if ( state == 0 ) {
//    enabled = true;
//  } else {
//    enabled = false;
//  }
//  ui.line_edit_master->setEnabled(enabled);
//  ui.line_edit_host->setEnabled(enabled);
//  //ui.line_edit_topic->setEnabled(enabled);
//}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
//void MainWindow::updateLoggingView() {
//        ui.view_logging->scrollToBottom();
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "op3_webots_gui");
//    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();

    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    bool checked = settings.value("use_environment_variables", false).toBool();

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "op3_webots_gui");

    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());


}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace op3_webots_gui

