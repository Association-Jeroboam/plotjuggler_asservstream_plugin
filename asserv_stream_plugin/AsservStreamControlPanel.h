#ifndef ASSERV_STREAM_CONTROL_PANEL_H
#define ASSERV_STREAM_CONTROL_PANEL_H

#include <QObject>
#include <QtPlugin>
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <map>
#include "PlotJuggler/statepublisher_base.h"
#include "ui_asservstreamcontrolpanel.h"
#include "AsservStream_uartDecoder.h"


class AsservStreamControlPanel: public QMainWindow
{
    Q_OBJECT
public:
    explicit AsservStreamControlPanel(Ui_AsservStreamControlPanel *ui, AsservStream_uartDecoder *uartDecoder, QSerialPort * device, int logFd = -1);
    ~ AsservStreamControlPanel(){}

    void closeEvent ( QCloseEvent* ) override{
        emit closed();
    }


    void updateConfig(float *tab, int nbValue);

public slots:
    void on_reset_btn_clicked();
    void on_motor_enable_btn_clicked();
    void on_motor_disable_btn_clicked();
    void on_distAngle_enable_btn_clicked();
    void on_distAngle_disable_btn_clicked();
    void on_vitesse_gauche_update_btn_clicked();
    void on_vitesse_droite_update_btn_clicked();
    void on_distance_update_btn_clicked();
    void on_angle_update_btn_clicked();

    void on_angle_acc_update_btn_clicked();
    void on_distance_acc_update_btn_clicked();

    void on_robot_speed_lin_cmd_btn_clicked();
    void on_robot_speed_ang_cmd_btn_clicked();
    void on_left_whell_speed_cmd_btn_clicked();
    void on_right_whell_speed_cmd_btn_clicked();

    void on_pos_cmd_btn_clicked();
    void on_angle_cmd_btn_clicked();
    void on_goto_cmd_btn_clicked();

    void on_get_config_btn_clicked();
    void on_update_config_btn_clicked();

    void on_goto_test_btn_clicked();

    void on_spinboxLeftValueChanged(int i);
    void on_spinboxRightValueChanged(int i);



signals:
    void closed();


private:

    void send(char *buffer, size_t length);
    Ui_AsservStreamControlPanel *ui_;
    QSerialPort * m_device;
    int logFd_;
    AsservStream_uartDecoder *uartDecoder;

    int leftSpinBoxValue;
    int rightSpinBoxValue;
    float leftKp[3];
    float leftKi[3];
    float leftSpeedRange[3];
    float rightKp[3];
    float rightKi[3];
    float rightSpeedRange[3];
};



#endif // ASSERV_STREAM_CONTROL_PANEL_H
