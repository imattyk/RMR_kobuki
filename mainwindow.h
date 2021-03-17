#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include<windows.h>
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include "ckobuki.h"
#include "rplidar.h"
/*#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"*/

typedef struct{
    double x;
    double y;
    double fi;
    double dist;
}worldPoint;

typedef struct{
    int x;
    int y;
    int value;
}MapPoint;

typedef struct{
    int mapsize = 400;
    int map[400][400];
    double resolution = 40 ; //  mm
    MapPoint mstart;
    MapPoint mfinish;
    worldPoint wstart;
    worldPoint wfinish;
}MapType;

typedef struct{
    double Kc = 140;
    double Ks = 600;// rozdiel je v metroch rychlost chcem v mm >> chyba v metroch >> 100vky mm
    double circ;
    double speed;
    const double max_trans_speed = 600;
}Regstruct;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);

    void processThisRobot();
    HANDLE robotthreadHandle; // handle na vlakno
    DWORD robotthreadID;  // id vlakna
    static DWORD WINAPI robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
    HANDLE laserthreadHandle; // handle na vlakno
    DWORD laserthreadID;  // id vlakna
    static DWORD WINAPI laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    unsigned int rob_slen;

    //----- moje funkcie
    void odometria();
    worldPoint loadTarget();
    double getAngle(double x1, double y1, double x2, double y2);
    double getDistance(double x1, double y1, double x2, double y2);
    void updateError();
    void regulator();
    void navigation();
    void rotateRobotLeft();
    void rotateRobotRight();
    void createMap(MapType *map);
    void fillMap(double distance, double angle);
    void writeMap(MapType map, string name);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

private:
     JOYINFO joystickInfo;
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     CKobuki robot;
     TKobukiData robotdata;
     int datacounter;
     QTimer *timer;
     double prewEncoderL,prewEncoderR,startEncL,startEncR,distanceL,distanceR,pDistanceL,pDistanceR = 0.0;
     double fi,prewFi,x,y,fiAbsolute = 0.0;
     boolean firstRun = true;
     boolean isNavigation = false;
     boolean isStart = false;
     boolean isRotate = false;
     worldPoint newTarget;
     worldPoint finalTarget;
     double angleError = 0.0;
     Regstruct reg;
     MapType mapData;
     boolean mapingState = false;

public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo


};




#endif // MAINWINDOW_H
