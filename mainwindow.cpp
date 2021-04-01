#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#define _USE_MATH_DEFINES
#include <QtMath>
#include <QDebug>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";
    //cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
    x=y=0.0; fi=prewFi=fiAbsolute=0;
  //  timer = new QTimer(this);
 //   connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera=false;
    createMap(&mapData);



    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect(20,120,700,500);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);

  /*  if(useCamera==true)
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );
        painter.drawImage(rect,image.rgbSwapped());
    }
    else*/
    {
        if(updateLaserPicture==1)
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                /*  int dist=rand()%500;
            int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-k)*3.14159/180.0))+rect.topLeft().x();
            int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-k)*3.14159/180.0))+rect.topLeft().y();*/
                int dist=copyOfLaserData.Data[k].scanDistance/20;
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
                if(rect.contains(xp,yp))
                    painter.drawEllipse(QPoint(xp, yp),2,2);

                //updatni nasu mapu s datami z ladaru

                if(!isRotate && counter == 0){
                    updateMap(copyOfLaserData.Data[k].scanDistance, 360-copyOfLaserData.Data[k].scanAngle);
                }
            }
        }
    }
}

void  MainWindow::setUiValues(double robotX,double robotY,double robotFi, int counter)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
     if(counter == 0){
         ui->label_6->setText("Mapping: on");
     }else{
         ui->label_6->setText("Mapping: off");
     }


}


void MainWindow::processThisRobot()
{
    odometria();
    updateError();

    if(datacounter%400 == 0 && mapingState){
         writeMapToTxt(mapData,"map");
     }

    if(datacounter%2 == 0){
        regulator();
    }

    if(datacounter%5)
    {
        emit uiValuesChanged(x,y,fiAbsolute, counter);
    }
    datacounter++;

}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia



}

void MainWindow::on_pushButton_9_clicked() //start button
{
    prewEncoderL = prewEncoderR = distanceL = distanceR = 0.0;
    fi = fiAbsolute = prewFi = 0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    laserthreadHandle=CreateThread(NULL,0,laserUDPVlakno, (void *)this,0,&laserthreadID);
    robotthreadHandle=CreateThread(NULL,0, robotUDPVlakno, (void *)this,0,&robotthreadID);
    /*  laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
      robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);*/
    connect(this,SIGNAL(uiValuesChanged(double,double,double, int)),this,SLOT(setUiValues(double,double,double, int)));

}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    std::vector<unsigned char> mess=robot.setTranslationSpeed(250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_3_clicked() //back
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_6_clicked() //left
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(3.14159/4);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_5_clicked()//right
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(-3.14159/4);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::laserprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;



    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(1)
    {
        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, (int*)&las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));


        processThisLidar(measure);




    }
}


void MainWindow::robotprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;



    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    Sleep(100);
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other,(int*) &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        //struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);
        if(!firstRun){
            prewEncoderL = robotdata.EncoderLeft;
            prewEncoderR = robotdata.EncoderRight;
        }

        int returnval=robot.fillData(robotdata,(unsigned char*)buff);
        if(firstRun) {
            prewEncoderL = robotdata.EncoderLeft;
            prewEncoderR = robotdata.EncoderRight;
            firstRun = false;
        }

        if(returnval==0)
        {
            processThisRobot();
        }


    }
}




void MainWindow::on_pushButton_clicked()
{
    if(useCamera==true)
    {
        useCamera=false;
        timer->stop();
        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera=true;
        timer->start(30);
        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}

//---------------- moje funckie
void MainWindow::odometria(){

    //qDebug() << QString::number(fi) + " first";

    //distance je vlastne len vzdialenost aku presiel encoder v tom danom cykle v ktorom ju davam
    //ona sa kompounduje, teda integruje dole priamo ked pocitame x/y suradnicu

    // Lave koleso
    if(prewEncoderL-robotdata.EncoderLeft >(60000)){
        distanceL = robot.getTick()*(robotdata.EncoderLeft-prewEncoderL + 65535);
    }else if(prewEncoderL-robotdata.EncoderLeft <(-60000)){
        distanceL = robot.getTick()*(robotdata.EncoderLeft-prewEncoderL - 65535);
    }else distanceL = robot.getTick()*(robotdata.EncoderLeft - prewEncoderL);

    // prave koleso
    if(prewEncoderR-robotdata.EncoderRight >(60000)){
        distanceR = robot.getTick()*(robotdata.EncoderRight-prewEncoderR + 65535);
    }else if(prewEncoderR-robotdata.EncoderRight<(-60000)){
        distanceR = robot.getTick()*(robotdata.EncoderRight-prewEncoderR - 65535);
    }else distanceR = robot.getTick()*(robotdata.EncoderRight- prewEncoderR);

    // uhol natocenia, z pohybu kolies
    fi = prewFi + (distanceR - distanceL)/(1.0*robot.getB());

    fi = fmod(fi,(2*M_PI));

    //absolutna hodnota natocenia
    if(fi < 0) fiAbsolute = (2*M_PI) + fi;
    else fiAbsolute = fi;

    //x a y vzorce
    if(distanceL == distanceR){
        x = x + distanceR*cos(prewFi);
        y = y + distanceR*sin(prewFi);
    }else{
        x = x + ((robot.getB()*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(sin(fi)-sin(prewFi)));
        y = y - ((robot.getB()*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(cos(fi)-cos(prewFi)));
    }

    prewFi = fi;

}

worldPoint MainWindow::loadTarget(){
    worldPoint target;
    target.x = (ui->lineEdit_5->text().toDouble());
    target.y = (ui->lineEdit_6->text().toDouble());
    return target;
}

// START NAVIGATION button
void MainWindow::on_pushButton_10_clicked()
{
    isNavigation = true;
    isStart = true;
    finalTarget = loadTarget();
    newTarget = finalTarget;
    mapingState = true;
    counter = 1;
}

double MainWindow::getAngle(double x1, double y1, double x2, double y2){
    return atan2(y2-y1,x2-x1);
}

double MainWindow::getDistance(double x1, double y1, double x2, double y2){
   return (double)sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

void MainWindow::updateError(){

    //pocita a upravuje pre newTarget
    //---------------------
    newTarget.fi = getAngle(x,y,newTarget.x,newTarget.y);
    newTarget.dist = getDistance(x,y,newTarget.x,newTarget.y);
    //---------------------

    if(newTarget.fi<0.0) newTarget.fi = (2*M_PI) + newTarget.fi;
    else newTarget.fi =  newTarget.fi;

    //newTarget.fi = fmod(newTarget.fi+M_PI_2,(2*M_PI));

    angleError =  newTarget.fi - fiAbsolute;
    if(newTarget.fi > 5.5 && fiAbsolute < 0.5){
       angleError = -((2*M_PI)-angleError);
    }
    if(newTarget.fi < 0.5 && fiAbsolute > 5.5){
       angleError = (2*M_PI)+angleError;
    }

    //qDebug() << "zelana: " + QString::number(newTarget.fi);
    //qDebug() << "error: " + QString::number(angleError);
    //qDebug() << "========================";
}

void MainWindow::regulator(){
    if(isStart){
        if(abs(angleError)>(M_PI_4)){
            if(angleError > 0){
                rotateRobotLeft();
                isRotate = true;
            } else if (angleError < 0){
                rotateRobotRight();
                isRotate = true;
            }

        }else {
            MainWindow::on_pushButton_4_clicked();  // Stop
            isRotate = false;
        }

       //Ak sa ocitneme v mensej vzdialenosti ako 0.05 od ciela, zaokruhlime ze sme tam, huraa
       if(newTarget.dist < 0.05){
            counter = 1;
            isStart = false;
            isRotate = false;
            MainWindow::on_pushButton_11_clicked();  // Stop
        }


       //ak sa teda vysavac netoci tak moze ist
       if(!isRotate && isStart){
           //qDebug() << newTarget.dist;
           // Rozbeh po kruznici
           //-------------------------------
           reg.circ = reg.Kc/(angleError);
           reg.speed = reg.Ks*newTarget.dist;
           //-------------------------------

           if(counter != 0){
               counter++;
               if(counter == 40){
                   counter = 0;
               }
           }


           if(reg.speed > reg.max_trans_speed) reg.speed = reg.max_trans_speed;

           if(isinf(reg.circ)) reg.circ = 32000;

           std::vector<unsigned char> mess=robot.setArcSpeed((int)reg.speed,(int)reg.circ);
           if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
           {

           }
       }
    }

}

void MainWindow::navigation(){

}

//STOP button
void MainWindow::on_pushButton_11_clicked()
{
    on_pushButton_4_clicked();
    isStart = false;
}

void MainWindow::rotateRobotLeft(){
   on_pushButton_6_clicked(); //left
}
void MainWindow::rotateRobotRight(){
   on_pushButton_5_clicked(); //right
}

void MainWindow::createMap(MapType *map){
    for(int i = 0; i< map->mapsize;i++){
        for(int j = 0; j<map->mapsize; j++){

            map->map[i][j] = 0;
        }

    }
}

void MainWindow::updateMap(double distance, double angle){
     int xm,ym;
     int ofset = mapData.mapsize/2;
    qDebug() << "angle: " + QString::number(angle) + "  Distance: " + QString::number(distance);

    if((distance <= 1500.0) && (distance != 0.0)){
        xm = (int)((((x)*1000.0) + (distance*cos((angle*M_PI/180) + fiAbsolute)))/mapData.resolution);
        ym = (int)((((y)*1000.0) + (distance*sin((angle*M_PI/180) + fiAbsolute)))/mapData.resolution);
        mapData.map[xm+ofset][ym+ofset] = 1;
    }
}

void MainWindow::writeMapToTxt(MapType map, string name){
    ofstream file;
    file.open(name+".txt", ios::trunc);
    for(int i=0; i<map.mapsize; i++){
       if(!(i==0)) file<<endl;
        for(int j=0; j<map.mapsize; j++){
            file<<map.map[i][j];
        }
    }
    file.close();

}

