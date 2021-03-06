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

    lidarData.minDcrit = robotSizeData.diameter;

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
    QPen pero,redPen,magentaPen,yellowPen,bluePen;

    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    redPen.setStyle(Qt::SolidLine); //styl pera - plna ciara
    redPen.setWidth(3);//hrubka pera -3pixely
    redPen.setColor(Qt::red);//farba je cervena

    magentaPen.setStyle(Qt::SolidLine); //styl pera - plna ciara
    magentaPen.setWidth(3);//hrubka pera -3pixely
    magentaPen.setColor(Qt::magenta);//farba je magenta

    yellowPen.setStyle(Qt::SolidLine); //styl pera - plna ciara
    yellowPen.setWidth(3);//hrubka pera -3pixely
    yellowPen.setColor(Qt::yellow);//farba je zlta

    bluePen.setStyle(Qt::SolidLine); //styl pera - plna ciara
    bluePen.setWidth(3);//hrubka pera -3pixely
    bluePen.setColor(Qt::cyan);//farba je bledo modra

    QRect rect(20,120,700,500);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);



    if(updateLaserPicture==1)
    {
        //mutex.lock();
        updateLaserPicture=0;
        painter.setPen(pero);


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

            if(!isRotate && mappingCounter == 0){
                updateMap(copyOfLaserData.Data[k].scanDistance, 360-copyOfLaserData.Data[k].scanAngle);
            }
        }

        //-------------------------------------------------------------------------------------------

        // vykreslenie bodu minLidarDist
        int dist=lidarData.minLidarDist/20;
        int xp=rect.width()-(rect.width()/2+dist*2*sin((360 - lidarData.minAngle)*3.14159/180.0))+rect.topLeft().x();
        int yp=rect.height()-(rect.height()/2+dist*2*cos((360 - lidarData.minAngle)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(xp,yp)){
            painter.setPen(yellowPen);
            painter.drawEllipse(QPoint(xp, yp),4,4);
        }

        // vykreslenie bodu maxLidarDistL
        int maxLidarDistL=lidarData.maxLidarDistL/20;
        int maxpl=rect.width()-(rect.width()/
                         2+maxLidarDistL*2*sin((360 - lidarData.maxLidarAngleL)*3.14159/180.0))+rect.topLeft().x();
        int maypl=rect.height()-(rect.height()/2+maxLidarDistL*2*cos((360 - lidarData.maxLidarAngleL)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(maxpl,maypl)){
            painter.setPen(redPen);
            painter.drawEllipse(QPoint(maxpl, maypl),4,4);//vykreslime kruh s polomerom 2px
        }
        // vykreslenie bodu maxLidarDistR
        int maxLidarDistR=lidarData.maxLidarDistR/20;
        int maxpr=rect.width()-(rect.width()/
                         2+maxLidarDistR*2*sin((360 - lidarData.maxLidarAngleR)*3.14159/180.0))+rect.topLeft().x();
        int maypr=rect.height()-(rect.height()/2+maxLidarDistR*2*cos((360 - lidarData.maxLidarAngleR)*3.14159/180.0))+rect.topLeft().y();
        if(rect.contains(maxpr,maypr)){
            painter.setPen(bluePen);
            painter.drawEllipse(QPoint(maxpr, maypr),4,4);//vykreslime kruh s polomerom 2px
        }
    }
    //mutex.unlock();
}

void  MainWindow::setUiValues(double robotX,double robotY,double robotFi, int mappingCounter)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
     if(mappingCounter == 0){
         ui->label_6->setText("Mapping: on");
     }else{
         ui->label_6->setText("Mapping: off");
     }


}


void MainWindow::processThisRobot()
{
    odometria();
    updateError();

    if(isMapNavigation){
        mapNavigation();
    }

    if(isLidarNavigation && datacounter % 2 == 0){
        updateLidarNavigation();
    }


    if(isLidarNavigation && datacounter % 50 == 0){
        lidarNavigation();
    }


    if(datacounter%400 == 0 && mapingState){
        writeMapToTxt("map", &mapData);
     }

    if(datacounter%4 == 0){
        regulator();
    }

    if(datacounter%5)
    {
        emit uiValuesChanged(x,y,fiAbsolute, mappingCounter);
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
    isLidarNavigation = true;
    isStart = true;
    finalTarget = loadTarget();
    newTarget = finalTarget;
    isGoToPoint = true;
    mapingState = true;
    mappingCounter = 1;
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

    finalTarget.fi = getAngle(x,y,finalTarget.x,finalTarget.y);
    finalTarget.dist = getDistance(x,y,finalTarget.x,finalTarget.y);
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

}

void MainWindow::regulator(){
    if(isStart){
        if(abs(angleError)>(M_PI_4/6)){
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
            mappingCounter = 1;
            isStart = false;
            isRotate = false;
            isGoToPoint = false;
            if(finalTarget.dist > 0.05){
                newTarget = finalTarget;
                isStart = true;
                isGoToPoint = true;
            }

            on_pushButton_4_clicked(); //stop
        }


       //ak sa teda vysavac netoci tak moze ist
       if(!isRotate && isStart){
           // Rozbeh po kruznici
           //-------------------------------
           reg.circ = reg.Kc/(angleError);
           reg.speed = reg.Ks*newTarget.dist;
           //-------------------------------

           if(mappingCounter != 0){
               mappingCounter++;
               if(mappingCounter == 40){
                   mappingCounter = 0;
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

void MainWindow::lidarNavigation(){

   /* if(isWallFollow){
        newTarget = finalTarget;
        worldPoint tmpPoint = wallFollowing(); // TODO
        if(tmpPoint.dist != 123456){
            newTarget = tmpPoint;
            isStart = true;
            isGoToPoint = true;
            firstPathBlockedCycle = true;
        }
    }*/

    if(isGoToPoint && isPathBlocked()){
        isStart = false;
        isGoToPoint = false;
        //on_pushButton_4_clicked();
        //firstPathBlockedCycle = false;
        worldPoint tmpPoint = wallDetection();
        if(tmpPoint.dist != 123456) {
            if(tmpPoint.dist < navigateData.minDist2Fintarget){
                navigateData.minDist2Fintarget = tmpPoint.dist;
            }
            newTarget = tmpPoint;
            isStart = true;
            isGoToPoint = true;
            firstPathBlockedCycle = true;
        }else{
            //nevieme najst kraj steny
            qDebug() << "nevie naajst kraj";
            isStart = true;
            isGoToPoint = true;
        }
    }

}

//STOP button
void MainWindow::on_pushButton_11_clicked()
{
    on_pushButton_4_clicked();
    isStart = false;
    isMapNavigation = false;
    isGoToPoint = false;
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

    if((distance <= 1500.0) && (distance != 0.0)){
        xm = (int)((((x)*1000.0) + (distance*cos((angle*M_PI/180) + fiAbsolute)))/mapData.resolution);
        ym = (int)((((y)*1000.0) + (distance*sin((angle*M_PI/180) + fiAbsolute)))/mapData.resolution);
        mapData.map[xm+ofset][ym+ofset] = 1;
    }
}

void MainWindow::writeMapToTxt(string name, MapType* map){
    ofstream file;
    file.open(name+".txt", ios::trunc);
    for(int i=0; i<map->mapsize; i++){
       if(!(i==0)) file<<endl;
        for(int j=0; j<map->mapsize; j++){
            file<<map->map[i][j];
        }
    }
    file.close();

}

//Map navigation button
void MainWindow::on_pushButton_12_clicked()
{
    finalTarget = loadTarget();
    mapName = ui->lineEdit_7->text();
    flood();
    isMapNavigation = true;
    mapingState = true;
}

void MainWindow::mapNavigation(){


    if(!path.empty() && !isStart){
        newTarget.x = path.front().x;
        newTarget.y = path.front().y;
        path.pop();
        isStart = true;
    }
    if(path.empty()){
        isMapNavigation = false;
    }
}

void MainWindow::flood(){
    if(mapName == ""){

    }else{
        navigationMapPtr = loadMapFromTxt(mapName.toStdString());
    }
    navigationMapPtr = enlargeWalls(navigationMapPtr);
    writeMapToTxt("enlarged_map", navigationMapPtr);

    navigationMapPtr->wfinish = createWorldPoint(finalTarget.x,finalTarget.y);
    navigationMapPtr->wstart = createWorldPoint(x,y);

    navigationMapPtr = startTheFlood(navigationMapPtr);
    //writeMapToTxt("flooded_map", navigationMapPtr);
    writeMapToCsv("flooded_map",navigationMapPtr);
    mappath = pathFinder(navigationMapPtr);
    path = map2worldPath(mappath);
}

MapType* MainWindow::loadMapFromTxt(string name){
    fstream file;
    string line;
    MapType *tmpMapPtr = new MapType();
    file.open(name + ".txt", ios::in);
    for(int i= 0; i < tmpMapPtr->mapsize; i++){
        getline(file,line);
        for(int j=0; j<= tmpMapPtr->mapsize; j++){
            if(j!=tmpMapPtr->mapsize) {
                tmpMapPtr->map[i][j] = line[j] - '0';
         }
        }
    }
    file.close();
    return tmpMapPtr;
}

MapType* MainWindow::enlargeWalls(MapType* map){
    int enlargeSize = ((robotSizeData.r+200)/map->resolution);
    //We create copy of our map on the heap using 'new', creating MapType datatype on stack caused program to exceed the stack memory allocation capacity, thus crash :(
    MapType *copyMapPtr = new MapType();
    *copyMapPtr = *map;
    for(int i=1;i<map->mapsize-enlargeSize;i++){
        for(int j=1;j<map->mapsize-enlargeSize;j++){
            if((map->map[i-1][j] == 1) && (map->map[i][j] == 0)){
                for(int shift = enlargeSize; shift>0; shift--){
                    for(int counter = enlargeSize;counter>0;counter--){
                        copyMapPtr->map[i+counter][((j-shift)+enlargeSize/2)] = 1;
                    }
                    for(int counter = enlargeSize;counter>0;counter--){
                        copyMapPtr->map[i-counter][((j-shift)+enlargeSize/2)] = 1;
                    }
                }
            }
            if((map->map[i][j-1] == 1) && (map->map[i][j] == 0)){
                for(int shift = enlargeSize; shift>0; shift--){
                    for(int counter = enlargeSize;counter>0;counter--){
                        copyMapPtr->map[(i-shift)+enlargeSize/2][j+counter] = 1;
                        }
                    for(int counter = enlargeSize;counter>0;counter--){
                        copyMapPtr->map[(i-shift)+enlargeSize/2][j-counter] = 1;
                    }
                }
            }
        }
    }
    return copyMapPtr;
}

worldPoint MainWindow::createWorldPoint(double x, double y){
    worldPoint point;
    point.x = x;
    point.y = y;
    return point;
}

MapPoint MainWindow::createMapPoint(int x, int y, int value){
    MapPoint point;
    point.x = x;
    point.y = y;
    point.value = value;
    return point;
}

MapType* MainWindow::startTheFlood(MapType* map){
    MapType *copyMapPtr = new MapType();
    *copyMapPtr = *map;
    list<MapPoint> points2go;
    copyMapPtr->mfinish = world2mapConverter(copyMapPtr->wfinish.x,copyMapPtr->wfinish.y);
    copyMapPtr->mfinish.value = 2;
    copyMapPtr->mstart = world2mapConverter(copyMapPtr->wstart.x,copyMapPtr->wstart.y);
    copyMapPtr->mstart.value = 95201;
    int dirX[4] = {-1,0,0,1};
    int dirY[4] = {0,-1,1,0};

    copyMapPtr->map[copyMapPtr->mstart.x][copyMapPtr->mstart.y] = copyMapPtr->mstart.value;
    points2go.push_back(copyMapPtr->mfinish);
    points2go.begin()->value = 3;
    copyMapPtr->map[points2go.begin()->x][points2go.begin()->y] = points2go.begin()->value-1; //end point


    while(!points2go.empty()){
        // checking 4 neighbours
        for(int i=0;i<4;i++){
            if(copyMapPtr->map[(points2go.begin()->x)+dirX[i]][(points2go.begin()->y)+dirY[i]] == 95201){
                return copyMapPtr; //We found the way
            }
            if(copyMapPtr->map[(points2go.begin()->x)+dirX[i]][(points2go.begin()->y)+dirY[i]] == 0){
                // insert new point at the end of the list with new coordinates
                points2go.push_back(createMapPoint(points2go.begin()->x + dirX[i], points2go.begin()->y +dirY[i], (points2go.begin()->value + 1)));

                //set value of next step in flood
                copyMapPtr->map[(points2go.begin()->x)+dirX[i]][(points2go.begin()->y)+dirY[i]] = points2go.begin()->value;
            }
        }
        points2go.pop_front(); // pop the last checked point
    }

    return copyMapPtr;
}

MapPoint MainWindow::world2mapConverter(double x_w, double y_w){
    MapPoint tmpPoint;
    int ofset = mapData.mapsize/2;
    tmpPoint.x = (int)(x_w*1000.0/40.0);
    tmpPoint.y = (int)(y_w*1000.0/40.0);

    tmpPoint.x += ofset;
    tmpPoint.y += ofset;
    return tmpPoint;
}

worldPoint MainWindow::map2worldConverter( int x_m, int y_m){
    int ofset = mapData.mapsize/2;
    worldPoint tmpPoint;
    tmpPoint.x = ((double)(x_m-ofset))*40.0/1000.0;
    tmpPoint.y = ((double)(y_m-ofset))*40.0/1000.0;

    return tmpPoint;
}


void MainWindow::writeMapToCsv(string name, MapType *map){
    ofstream file;
    file.open(name+".csv", ios::trunc);
    for(int i=0; i<map->mapsize; i++){
       if(!(i==0)) file<<endl;
        //file << endl;
        for(int j=0; j<map->mapsize; j++){
            if(j!=map->mapsize-1) file<<map->map[i][j]<<",";
            else file<<map->map[i][j];
        }
    }
    file.close();

}

list<MapPoint> MainWindow::pathFinder(MapType *map){
    int dirY[4] = {0,1,0,-1};
    int dirX[4] = {-1,0,1,0};
    int prewDir = 0;
    int lowVal,lowDir;
    boolean firstMove = true;
    list<MapPoint> points2go;
    list<MapPoint> pathPoints;
    MapPoint position = map->mstart;
    MapPoint prewPosition;

    lowVal = 999999; //some huuge value that will be higher than anything for start
     while(position.value != map->mfinish.value){
        for(int i=0;i<4;i++){
           //if there is no wall and is the smallest
           if(map->map[(position.x)+dirX[i]][(position.y)+dirY[i]] > 1 && map->map[(position.x+dirX[i])][(position.y +dirY[i])] < lowVal){

               if(!points2go.empty()) {
                   points2go.pop_front();
               }

               points2go.push_back(createMapPoint(position.x + dirX[i], position.y +dirY[i], map->map[(position.x+dirX[i])][(position.y +dirY[i])]));
               //set lowest value of neighbours
               lowVal = points2go.begin()->value;
               //set direction
               lowDir = i;
           }
       }

       // move in the direction
       prewPosition = position;
       position.x = points2go.begin()->x;
       position.y = points2go.begin()->y;
       position.value = points2go.begin()->value;

       //if direction was changed and this was not our first move
       if(prewDir != lowDir && !firstMove){
           // add path point
           pathPoints.push_back(prewPosition);
           // set new direction
           prewDir = lowDir;
       }
       firstMove = false;

       // we found the end
       if(position.value == map->mfinish.value) pathPoints.push_back(position);
    }

   return pathPoints;
}

queue<worldPoint> MainWindow::map2worldPath(list<MapPoint> mappath){
    queue<worldPoint> wrldpath;
    int count = 0;
    while(!mappath.empty()){
            wrldpath.push(map2worldConverter(mappath.begin()->x,mappath.begin()->y));
            mappath.pop_front();
            count++;
    }
    return wrldpath;
}

bool MainWindow::isPathBlocked(){
    if(lidarData.minPoint) return TRUE;
    else return FALSE;
}

double MainWindow::fixAngle(double fi){

    if(fi>0.0) fi = (2*M_PI) - fi;
    else fi = - fi;
    fi = fmod(fi+M_PI_2,(2*M_PI));
    return fi;
}

worldPoint MainWindow::wallDetection(){
    worldPoint leftsecurePoint,rightsecurePoint;
    worldPoint errpoint;
    errpoint.dist = 123456;

    if(lidarData.maxPointL){
      leftsecurePoint = findSecurePointL(lidarData.maxXL,lidarData.maxYL);

      leftsecurePoint.dist = getDistance(x,y,leftsecurePoint.x,leftsecurePoint.y) + getDistance(leftsecurePoint.x,leftsecurePoint.y,finalTarget.x,finalTarget.y);
    }
    if(lidarData.maxPointR){
      rightsecurePoint = findSecurePointR(lidarData.maxXR,lidarData.maxYR);

      rightsecurePoint.dist = getDistance(x,y,rightsecurePoint.x,rightsecurePoint.y) + getDistance(rightsecurePoint.x,rightsecurePoint.y,finalTarget.x,finalTarget.y);
    }

    if(lidarData.maxPointL){
           if(lidarData.maxPointR){
               if(leftsecurePoint.dist > rightsecurePoint.dist) {  return rightsecurePoint;}
                        else { return leftsecurePoint;}
           }
           else { return leftsecurePoint;}
     }else{
        if(lidarData.maxPointR) { return rightsecurePoint;}
            else return errpoint;
        }
}

worldPoint MainWindow::findSecurePointR(double edgePointX, double edgePointY){

    worldPoint securepoint;
    double b = (robotSizeData.r + 300)/1000;
    double edgeDist = getDistance(x,y,edgePointX,edgePointY);
    double edgeAngle = getAngle(x,y,edgePointX,edgePointY);
    securepoint.dist = sqrt(pow(edgeDist,2) - pow(b,2));
    securepoint.dist += 0.2;

    double alpha = asin(b/edgeDist);
    securepoint.fi = edgeAngle - alpha;

    securepoint.x = x + ((securepoint.dist)*cos(securepoint.fi));
    securepoint.y = y + ((securepoint.dist)*sin(securepoint.fi));

    return securepoint;
}

worldPoint MainWindow::findSecurePointL(double edgePointX, double edgePointY){
    worldPoint securepoint;
    double b = (robotSizeData.r + 300)/1000;
    double edgeDist = getDistance(x,y,edgePointX,edgePointY);
    double edgeAngle = getAngle(x,y,edgePointX,edgePointY);
    securepoint.dist = sqrt(pow(edgeDist,2) - pow(b,2));
    securepoint.dist += 0.2;

    double alpha = asin(b/edgeDist);
    securepoint.fi = edgeAngle + alpha;

    securepoint.x = x + ((securepoint.dist)*cos(securepoint.fi));
    securepoint.y = y + ((securepoint.dist)*sin(securepoint.fi));

    return securepoint;
}

void MainWindow::updateLidarNavigation(){

    double angleDiff;

    mutex.lock();
    lidarData.minLidarDist = HUGE_VAL;
    lidarData.minAngle = HUGE_VAL;
    lidarData.maxLidarDistL = -HUGE_VAL;
    lidarData.maxLidarAngleL = -HUGE_VAL;
    lidarData.maxLidarDistR = -HUGE_VAL;
    lidarData.maxLidarAngleR = -HUGE_VAL;

    //newTarget = loadTarget();
    //newTarget.fi= getAngle(x,y,newTarget.x,newTarget.y);
    //newTarget.dist = getDistance(x,y,newTarget.x,newTarget.y);
    //updateError();



    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {


        // detekcia prekazky
        angleDiff = (fiAbsolute - newTarget.fi)*180/M_PI;
        if (angleDiff < 0) angleDiff = 360 + angleDiff;

        ///zistujem prekazku na uhle voci bodu
       if (copyOfLaserData.Data[k].scanDistance < lidarData.minLidarDist && copyOfLaserData.Data[k].scanDistance < 1500.0 && ((fmod(copyOfLaserData.Data[k].scanAngle,360.0)) < 5  || (fmod(copyOfLaserData.Data[k].scanAngle,360.0)) > 355 )){
           if(angleDiff < 5 || angleDiff > 355){
               if(newTarget.dist*1000 > copyOfLaserData.Data[k].scanDistance){
                   lidarData.minLidarDist = copyOfLaserData.Data[k].scanDistance;
                   lidarData.minAngle = copyOfLaserData.Data[k].scanAngle;
               }
           }
       }

       //lava strana prekazky
       if(angleDiff < 5 || angleDiff > 355){
           if ((copyOfLaserData.Data[k].scanDistance > lidarData.maxLidarDistL && copyOfLaserData.Data[k].scanDistance < 1500.0 ) && ((fmod(copyOfLaserData.Data[k].scanAngle,360.0)) <= 360) && ((fmod(copyOfLaserData.Data[k].scanAngle,360.0)) >= 320)){
                 lidarData.maxLidarDistL = copyOfLaserData.Data[k].scanDistance;
                 lidarData.maxLidarAngleL = copyOfLaserData.Data[k].scanAngle;

            }
       }

       //prava strana prekazky
       if(angleDiff < 5 || angleDiff > 355){
           if ((copyOfLaserData.Data[k].scanDistance > lidarData.maxLidarDistR && copyOfLaserData.Data[k].scanDistance < 1500.0 ) && ((fmod(copyOfLaserData.Data[k].scanAngle,360.0)) <= 40) && ((fmod(copyOfLaserData.Data[k].scanAngle,360.0)) >= 0)){
                lidarData.maxLidarDistR = copyOfLaserData.Data[k].scanDistance;
                lidarData.maxLidarAngleR = copyOfLaserData.Data[k].scanAngle;
            }
       }


    }

    //--------------------------------------------------------------------------------------

    // vypocet svetovych suradnic bodu minLidarDist
    if(!isinf(lidarData.minLidarDist) && !isinf(lidarData.minAngle)){
        lidarData.forminAngle = fmod(((360 - lidarData.minAngle)*M_PI/180) + fiAbsolute,(2.0*M_PI));  // uhol zvierany s x,y suradnicou robota vo svete
        lidarData.minX = x + ((lidarData.minLidarDist/1000.0)*cos(lidarData.forminAngle));
        lidarData.minY = y + ((lidarData.minLidarDist/1000.0)*sin(lidarData.forminAngle));
        lidarData.minPoint = TRUE;

    }else lidarData.minPoint = FALSE;

    // vypocet svetovych suradnic bodu maxL (kraj steny lavy)
    if(!isinf(lidarData.maxLidarDistL) && !isinf(lidarData.maxLidarAngleL)){
        lidarData.formAngleL = fmod((360 - lidarData.maxLidarAngleL) /180.0*M_PI + fiAbsolute,(2.0*M_PI));  // uhol zvierany s x,y suradnicou robota vo svete
        lidarData.maxXL = x + ((lidarData.maxLidarDistL/1000.0)*cos(lidarData.formAngleL));
        lidarData.maxYL = y + ((lidarData.maxLidarDistL/1000.0)*sin(lidarData.formAngleL));
        lidarData.maxPointL = TRUE;
    }else lidarData.maxPointL = FALSE;

    // vypocet svetovych suradnic bodu maxR (kraj steny pravy)
    if(!isinf(lidarData.maxLidarAngleR) && !isinf(lidarData.maxLidarDistR)){
        lidarData.formAngleR = fmod((360 - lidarData.maxLidarAngleR)/180.0*M_PI + fiAbsolute,(2.0*M_PI));  // uhol zvierany s x,y suradnicou robota vo svete
        lidarData.maxXR = x + ((lidarData.maxLidarDistR/1000.0)*cos(lidarData.formAngleR));
        lidarData.maxYR = y + ((lidarData.maxLidarDistR/1000.0)*sin(lidarData.formAngleR));
        lidarData.maxPointR = TRUE;

    }else lidarData.maxPointR = FALSE;

    mutex.unlock();


}
