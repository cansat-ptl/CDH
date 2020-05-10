#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qcustomplot.h>

mainwindow::mainwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::mainwindow)
{

    ui -> setupUi(this);

    // 3D
    auto *view = new Qt3DExtras::Qt3DWindow();
    container3D = createWindowContainer(view,this);
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity;
    Qt3DCore::QEntity *model = new Qt3DCore::QEntity(rootEntity);

    td_drawLine({ 0, 0, 0 }, { 10000, 0, 0 }, Qt::red, rootEntity); // X
    td_drawLine({ 0, 0, 0 }, { 0, 10000, 0 }, Qt::green, rootEntity); // Y
    td_drawLine({ 0, 0, 0 }, { 0, 0, 10000 }, Qt::blue, rootEntity); // Z
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(QColor(125, 125, 125));
    Qt3DRender::QMesh *modelMesh = new Qt3DRender::QMesh;
    QUrl data = QUrl::fromLocalFile("C:\\SRCS\\CDH\\model.stl");
    modelMesh->setMeshName("Device model");
    modelMesh->setSource(data);

    transform->setScale(0.1f);
    model->addComponent(transform);
    model->addComponent(modelMesh);
    model->addComponent(material);
    Qt3DRender::QCamera *camera = view -> camera();
    camera->setViewCenter(QVector3D(5, 5, 1));
    camera->setPosition(QVector3D(5,5,40.0f));
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("grey");
    light->setIntensity(0.8f);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(60, 0, 40.0f));
    lightEntity->addComponent(lightTransform);
    Qt3DExtras::QOrbitCameraController *camController = new
    Qt3DExtras::QOrbitCameraController(rootEntity);
    camController->setCamera(camera);
    view -> setRootEntity(rootEntity);

    // some design
    ui -> dashTerminal -> setReadOnly(1);
    QPalette paletteTerminal = ui -> dashTerminal -> palette();
    paletteTerminal.setColor(QPalette::Base, Qt::black);
    paletteTerminal.setColor(QPalette::Text, Qt::green);
    ui -> dashTerminal -> setPalette(paletteTerminal);

    // COM port init
    receiver = new QSerialPort(this);
    receiver -> setPortName(portNameR);
    receiver -> open(QSerialPort::ReadOnly);
    receiver -> setBaudRate(baudRateR.toInt());
    receiver -> setDataBits(QSerialPort::Data8);
    receiver -> setFlowControl(QSerialPort::NoFlowControl);
    receiver -> setParity(QSerialPort::NoParity);
    receiver -> setStopBits(QSerialPort::OneStop);
    receiver -> flush();
    QObject::connect(receiver, SIGNAL(readyRead()), this, SLOT(h_readSerial()));

    tracker = new QSerialPort(this);

    tracker -> setPortName(portNameT);
    tracker -> open(QSerialPort::WriteOnly);
    tracker -> setBaudRate(baudRateT.toInt());
    tracker -> setDataBits(QSerialPort::Data8);
    tracker -> setFlowControl(QSerialPort::NoFlowControl);
    tracker -> setParity(QSerialPort::NoParity);
    tracker -> setStopBits(QSerialPort::OneStop);
    tracker -> flush();

    transceiver = new QSerialPort(this);

    transceiver -> setPortName(portNameTrc);
    transceiver -> open(QSerialPort::WriteOnly);
    transceiver -> setBaudRate(baudRateTrc.toInt());
    transceiver -> setDataBits(QSerialPort::Data8);
    transceiver -> setFlowControl(QSerialPort::NoFlowControl);
    transceiver -> setParity(QSerialPort::NoParity);
    transceiver -> setStopBits(QSerialPort::OneStop);
    transceiver -> flush();

    // 3D trajectory scatter
    trajPlot -> setShadowQuality(QAbstract3DGraph::ShadowQualityNone);
    trajPlot -> addSeries(series);
    series -> setItemSize(0.06f);
    ui -> gpsStack -> insertWidget(0, trajContainer);
    trajContainer -> show();
    trajContainer -> setGeometry(0,0,615,290);
    QWidget temp;
    ui -> gpsStack -> insertWidget(1, &temp);
    temp.show();
    trajPlot -> axisY() -> setTitle("Altitude");
    trajPlot -> axisX() -> setTitle("Latitude");
    trajPlot -> axisZ() -> setTitle("Longitude");
    ui -> gpsStack -> widget(0) -> show();
    ui -> gpsStack -> setCurrentIndex(0);
    ui -> orientStack -> setCurrentIndex(0);

    init_data_plots(); init_3D_render(); init_WebSocket_connection();
}

void mainwindow::td_drawLine(const QVector3D& start, const QVector3D& end, const QColor& color, Qt3DCore::QEntity *_rootEntity)
{
    auto *geometry = new Qt3DRender::QGeometry(_rootEntity);

    // position vertices (start and end)
    QByteArray bufferBytes;
    bufferBytes.resize(3 * 2 * sizeof(float)); // start.x, start.y, start.end + end.x, end.y, end.z
    float *positions = reinterpret_cast<float*>(bufferBytes.data());
    *positions++ = start.x();
    *positions++ = start.y();
    *positions++ = start.z();
    *positions++ = end.x();
    *positions++ = end.y();
    *positions++ = end.z();

    auto *buf = new Qt3DRender::QBuffer(geometry);
    buf->setData(bufferBytes);

    auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(buf);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(2);
    geometry->addAttribute(positionAttribute); // We add the vertices in the geometry

    // connectivity between vertices
    QByteArray indexBytes;
    indexBytes.resize(2 * sizeof(unsigned int)); // start to end
    unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());
    *indices++ = 0;
    *indices++ = 1;

    auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
    indexBuffer->setData(indexBytes);

    auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
    indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(2);
    geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometry

    // mesh
    auto *line = new Qt3DRender::QGeometryRenderer(_rootEntity);
    line->setGeometry(geometry);
    line->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    auto *material = new Qt3DExtras::QPhongMaterial(_rootEntity);
    material->setAmbient(color);

    // entity
    auto *lineEntity = new Qt3DCore::QEntity(_rootEntity);
    lineEntity->addComponent(line);
    lineEntity->addComponent(material);
}

void mainwindow::h_readSerial()
{
    QStringList bufferSplit = serialBuffer.split("\n");
    if(bufferSplit.length() < 2)
    {
        serialData = receiver -> readAll();
        serialBuffer = serialBuffer + QString::fromStdString(serialData.toStdString());
        serialData.clear();
    }
    else
    {
        serialBuffer = "";
        parsedData = bufferSplit[0];
        mainwindow::h_updateData(parsedData.trimmed());
    }
}

void mainwindow::h_updateData(QString s)
{
   ui -> dashTerminal -> append("RCVD\n" + s + "\n");
   QString type = (QString)s[8] + (QString)s[9] + (QString)s[10] + (QString)s[11] + (QString)s[12] + (QString)s[13];
   QString callsign = (QString)s[0] + (QString)s[1] + (QString)s[2] + (QString)s[3] + (QString)s[4] + (QString)s[5] + (QString)s[6];
   ui -> callsign -> setText(callsign);
   if (type == "MAIN:N")
   {
        bool dmg = h_handleMain(s);
        if (dmg)
            ui -> dashTerminal -> append("DAMAGED!\n Packet number: " + QString::number(nMain));
        ui -> dashAltLabel -> setText(QString::number(altMain) + " m");
        ui -> dashPrsLabel -> setText(QString::number(prsMain) + " kPa");
        ui -> dashVbatLabel -> setText(QString::number(vbatMain) + " V");
        ui -> dashT1Label -> setText(QString::number(t1Main) + " C°");

        ui -> altPlot -> graph(0) -> addData(etMain, altMain);
        ui -> altPlot -> rescaleAxes();
        ui -> altPlot -> replot();

        ui -> prsPlot -> graph(0) -> addData(etMain, prsMain);
        ui -> prsPlot -> rescaleAxes();
        ui -> prsPlot -> replot();

        ui -> vbatPlot -> graph(0) -> addData(etMain, vbatMain);
        ui -> vbatPlot -> rescaleAxes();
        ui -> vbatPlot -> replot();

        ui -> tPlot -> graph(0) -> addData(etMain, t1Main);
        ui -> tPlot -> graph(1) -> addData(etMain, t2Main);
        ui -> tPlot -> rescaleAxes();
        ui -> tPlot -> replot();

        QString flagsMain = QString("%1").arg(flgMain,0,2);
        boolean value = 0;
        for (int k = 0; k < flagsMain.length(); k++)
        {
            if (flagsMain[k] == '0')
                value = 0;
            if (flagsMain[k] == '1')
                value = 1;
            d_changeFlag(k + 1, value);
        }
        ws_m_collected = 1;
        ws_sendMsg();

   }
   if (type == "ORIENT")
   {
        bool dmg = h_handleOrient(s);
        if (dmg)
            ui -> dashTerminal -> append("DAMAGED!\n Packet number: " + QString::number(nOrient));

        ui -> dashAxLabel -> setText(QString::number(axOrient) + " m/s²");
        ui -> dashAyLabel -> setText(QString::number(ayOrient) + " m/s²");
        ui -> dashAzLabel -> setText(QString::number(azOrient) + " m/s²");

        ui -> orientPlot -> graph(0) -> addData(etOrient, axOrient);
        ui -> orientPlot -> graph(1) -> addData(etOrient, ayOrient);
        ui -> orientPlot -> graph(2) -> addData(etOrient, azOrient);
        ui -> orientPlot -> rescaleAxes();
        ui -> orientPlot -> replot();
        ws_o_collected = 1;
        ws_sendMsg();
   }
   if (type == "GPS:N=")
   {
        bool dmg = h_handleGPS(s);
        if (dmg)
            ui -> dashTerminal -> append("DAMAGED!\n Packet number: " + QString::number(nGPS));
        double r1 = 6371200.0 + altStation.toDouble();
        double x1 = r1 * cos(latStation.toDouble()) * cos(lonStation.toDouble());
        double y1 = r1 * cos(latStation.toDouble()) * sin(lonStation.toDouble());

        double r2 = 6371200.0 + altGPS;
        double x2 = r2 * cos(latGPS) * cos(lonGPS);
        double y2 = r2 * cos(latGPS) * sin(lonGPS);

        double a = abs(y2 - y1);
        double b = abs(x2 - x1);
        double c = hypot(a, b) / 100.0;
        double d = hypot(c, altGPS - altStation.toDouble());

        double alpha = atan2(x2 - x1, y2 - y1) * 180 / M_PI;
        double beta = atan((altGPS - altStation.toDouble()) / c) * 180 / M_PI;

        if (alpha < 0)
        {
            alpha += 180;
            beta = 180 - beta;
            ui -> flipped -> setText("YES");
        }
        else
        {
            ui -> flipped -> setText("NO");
        }
        if (beta < 0)
        {
            beta = 0;
        }

        ui -> alphaLabel -> setText(QString::number(alpha) + "°");
        ui -> betaLabel -> setText(QString::number(beta) + "°");
        ui -> distLabel -> setText(QString::number(d) + " m");

        int rndAlpha = (int)alpha, rndBeta = (int)beta;
        cmd_writeToTracker("A=" + QString::number(rndAlpha) + ";" + "B=" + QString::number(rndBeta) + ";");
        d_updateScatter(latGPS, altGPS, lonGPS);
        ws_g_collected = 1;
        transform->setRotationX(pitchOrient); transform->setRotationY(yawOrient); transform->setRotationZ(rollOrient);
        ws_sendMsg();
   }
}

bool mainwindow::h_handleMain(QString s)
{
    int l = s.length();
    bool damaged = false;
    QString temp = "";
    int i = 0;
    for (i = 0; i < l; i++) {
        if (s[i] == '=') {
            if (s[i - 1] == 'N') {
                while (s[i + 1] != ';') {
                    temp += s[i + 1];
                    i++;
                    if (i > l) {
                        damaged = true;
                        break;
                    }
                }
                nMain = temp.toInt();
                temp = "";
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'E') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    etMain = temp.toInt();
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'A') {
                    if (s[i - 3] == 'B') {
                        if (s[i - 4] == 'V') {
                            while (s[i + 1] != ';') {
                                temp += s[i + 1];
                                i++;
                                if (i > l) {
                                    damaged = true;
                                    break;
                                }
                            }
                            vbatMain = temp.toDouble()/100.0;
                            temp = "";
                        }
                    }
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'L') {
                    if (s[i - 3] == 'A') {
                        while (s[i + 1] != ';') {
                            temp += s[i + 1];
                            i++;
                            if (i > l) {
                                damaged = true;
                                break;
                            }
                        }
                        altMain = temp.toDouble() / 10.0;
                        temp = "";
                    }
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'S') {
                if (s[i - 2] == 'R') {
                    if (s[i - 3] == 'P') {
                        while (s[i + 1] != ';') {
                            temp += s[i + 1];
                            i++;
                            if (i > l) {
                                damaged = true;
                                break;
                            }
                        }
                        prsMain = temp.toDouble() / 1000.0;
                        temp = "";
                    }
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == '1') {
                if (s[i - 2] == 'T') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    t1Main = temp.toDouble() / 10.0;
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == '2') {
                if (s[i - 2] == 'T') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    t2Main = temp.toDouble() / 10.0;
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'G') {
                if (s[i - 2] == 'L') {
                    if (s[i - 3] == 'F') {
                        while (s[i + 1] != ';') {
                            temp += s[i + 1];
                            i++;
                            if (i > l) {
                                damaged = true;
                                break;
                            }
                        }
                        flgMain = temp.toInt();
                        temp = "";
                    }
                }
            }
        }
    }
    return damaged;
}

bool mainwindow::h_handleOrient(QString s)
{
    int l = s.length();
    bool damaged = false;
    QString temp = "";
    int i = 0;
    for (i = 0; i < l; i++) {
        if (s[i] == '=') {
            if (s[i - 1] == 'N') {
                while (s[i + 1] != ';') {
                    temp += s[i + 1];
                    i++;
                    if (i > l) {
                        damaged = true;
                        break;
                    }
                }
                nOrient = temp.toInt();
                temp = "";
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'E') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    etOrient = temp.toInt();
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'X') {
                if (s[i - 2] == 'A') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    axOrient = temp.toDouble() / 10.0 * 9.81;
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'Y') {
                if (s[i - 2] == 'A') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    ayOrient = temp.toDouble() / 10.0 * 9.81;
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'Z') {
                if (s[i - 2] == 'A') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    azOrient = temp.toDouble() / 10.0 * 9.81;
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'H') {
                if (s[i - 2] == 'C') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    pitchOrient = temp.toDouble() / 10.0;
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'W') {
                if (s[i - 2] == 'A') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    yawOrient = temp.toDouble() / 10.0;
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'L') {
                if (s[i - 2] == 'L') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    rollOrient = temp.toDouble() / 10.0;
                    temp = "";
                }
            }
        }
    }
    return damaged;
}

bool mainwindow::h_handleGPS(QString s)
{
    int l = s.length();
    bool damaged = false;
    QString temp = "";
    int i = 0;
    for (i = 0; i < l; i++) {
        if (s[i] == '=') {
            if (s[i - 1] == 'N' && s[i - 2] != 'O') {
                while (s[i + 1] != ';') {
                    temp += s[i + 1];
                    i++;
                    if (i > l) {
                        damaged = true;
                        break;
                    }
                }
                nGPS = temp.toInt();
                temp = "";
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'E') {
                    while (s[i + 1] != ';') {
                        temp += s[i + 1];
                        i++;
                        if (i > l) {
                            damaged = true;
                            break;
                        }
                    }
                    etGPS = temp.toInt();
                    temp = "";
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'A') {
                    if (s[i - 3] == 'S') {
                        while (s[i + 1] != ';') {
                            temp += s[i + 1];
                            i++;
                            if (i > l) {
                                damaged = true;
                                break;
                            }
                        }
                        satGPS = temp.toInt();
                        temp = "";
                    }
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'A') {
                    if (s[i - 3] == 'L') {
                        while (s[i + 1] != ';') {
                            temp += s[i + 1];
                            i++;
                            if (i > l) {
                                damaged = true;
                                break;
                            }
                        }
                        latGPS = temp.toDouble();
                        temp = "";
                    }
                }
            }
        }  
        if (s[i] == '=') {
            if(s[i - 1] == 'N'){
                if (s[i - 2] == 'O') {
                    if (s[i - 3] == 'L') {
                        while (s[i + 1] != ';') {
                            temp += s[i + 1];
                            i++;
                            if (i > l) {
                                damaged = true;
                                break;
                            }
                        }
                        lonGPS = temp.toDouble();
                        temp = "";

                    }
                }
            }
        }
        if (s[i] == '=') {
            if (s[i - 1] == 'T') {
                if (s[i - 2] == 'L') {
                    if (s[i - 3] == 'A') {
                        while (s[i + 1] != ';') {
                            temp += s[i + 1];
                            i++;
                            if (i > l) {
                                damaged = true;
                                break;
                            }
                        }
                        altGPS = temp.toDouble() / 10.0;
                        temp = "";
                    }
                }
            }
        }
    }
    return damaged;
}

void mainwindow::d_updateScatter(double x, double y, double z)
{
    QScatterDataItem item;
    item.setX(x);
    item.setY(y);
    item.setZ(z);
    proxy -> addItem(item);
    trajContainer -> update();
}

void mainwindow::init_data_plots()
{
    ui -> altPlot -> addGraph();
    ui -> altPlot -> xAxis -> setLabel("seconds");
    ui -> altPlot -> yAxis -> setLabel("m");
    ui -> altPlot -> graph(0) -> setName("Altitude");
    ui -> altPlot -> legend -> setVisible(true);
    ui -> altPlot -> setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    ui -> prsPlot -> addGraph();
    ui -> prsPlot -> xAxis -> setLabel("seconds");
    ui -> prsPlot -> yAxis -> setLabel("kPa");
    ui -> prsPlot -> graph(0) -> setName("Pressure");
    ui -> prsPlot -> legend -> setVisible(true);
    ui -> prsPlot -> setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    ui -> vbatPlot -> addGraph();
    ui -> vbatPlot -> xAxis -> setLabel("seconds");
    ui -> vbatPlot -> yAxis -> setLabel("V");
    ui -> vbatPlot -> graph(0) -> setName("Battery voltage");
    ui -> vbatPlot -> legend -> setVisible(true);
    ui -> vbatPlot -> setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    ui -> tPlot -> addGraph();
    ui -> tPlot -> addGraph();
    ui -> tPlot -> graph(1) -> setPen(QPen(Qt::red));
    ui -> tPlot -> xAxis -> setLabel("seconds");
    ui -> tPlot -> yAxis -> setLabel("C°");
    ui -> tPlot -> graph(0) -> setName("Inside temperature");
    ui -> tPlot -> graph(1) -> setName("Outside temperature");
    ui -> tPlot -> legend -> setVisible(true);
    ui -> tPlot -> setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    ui -> orientPlot -> addGraph();
    ui -> orientPlot -> addGraph();
    ui -> orientPlot -> addGraph();
    ui -> orientPlot -> graph(0) -> setName("Acc. X");
    ui -> orientPlot -> graph(1) -> setName("Acc. Y");
    ui -> orientPlot -> graph(2) -> setName("Acc. Z");
    ui -> orientPlot -> graph(1) -> setPen(QPen(Qt::red));
    ui -> orientPlot -> graph(2) -> setPen(QPen(Qt::green));
    ui -> orientPlot -> xAxis -> setLabel("seconds");
    ui -> orientPlot -> yAxis -> setLabel("m/s²");
    ui -> orientPlot -> legend -> setVisible(true);
    ui -> orientPlot -> setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

mainwindow::~mainwindow()
{
    delete ui;
}

void mainwindow::resizeView(QSize size)
{
    container3D -> resize(size);
}

void mainwindow::td_resizeEvent(QResizeEvent * )
{
    resizeView(this->size());
}

void mainwindow::init_3D_render()
{
    ui->orientStack->insertWidget(1, container3D);
}

void mainwindow::on_comboBox_currentIndexChanged(const QString &ind)
{
    if (ind == "Altitude")
        ui -> mainStack -> setCurrentIndex(0);
    if (ind == "Pressure")
        ui -> mainStack -> setCurrentIndex(1);
    if (ind == "Battery voltage")
        ui -> mainStack -> setCurrentIndex(2);
    if (ind == "Temperature")
        ui -> mainStack -> setCurrentIndex(3);
}

void mainwindow::on_reconnectRecButton_clicked()
{
    receiver -> close();
    ui -> dashTerminal -> append("STNGS \nReceiver port closed.\n");
    portNameR = ui -> receiverCOMEdit -> text();
    baudRateR = ui -> receiverBaudEdit -> text();
    receiver -> setPortName(portNameR);
    receiver -> setBaudRate(baudRateR.toInt());
    receiver -> open(QSerialPort::ReadOnly);
    Sleep(uint(1000));
    ui -> dashTerminal -> append("STNGS \nReceiver port opened.\n");
}

void mainwindow::on_trackerApplyButton_clicked()
{
    latStation = ui -> latStationEdit -> text();
    lonStation = ui -> lonStationEdit -> text();
    altStation = ui -> altStationEdit -> text();
    ui -> dashTerminal -> append("STNGS \nStation coordinates changed.\n");
}

void mainwindow::on_pushButton_7_clicked()
{
    tracker -> close();
    ui -> dashTerminal -> append("STNGS \nTracker port closed.\n");
    portNameT = ui -> trackerCOMEdit -> text();
    baudRateT = ui -> trackerBaudEdit -> text();
    tracker -> setPortName(portNameT);
    tracker -> setBaudRate(baudRateT.toInt());
    tracker -> open(QSerialPort::WriteOnly);
    Sleep(uint(1000));
    ui -> dashTerminal -> append("STNGS \nTracker port closed.\n");
}

void mainwindow::cmd_writeToTerminal(QString s)
{
    QByteArray ba = s.toLatin1();
    transceiver -> write(ba);
    ui -> dashTerminal -> append("SENT \n" + s.trimmed() + "\n");
}

void mainwindow::cmd_writeToTracker(QString angles)
{
    angles.append( "\r");
    QByteArray ba = angles.toLatin1();
    tracker -> write(ba);
    ui -> dashTerminal -> append("TRCKR \n" + angles.trimmed() + "\n");
}

void mainwindow::d_changeFlag(int flag, bool value)
{
    switch (flag)
    {
        case 1:
            if (value)
            {
                ui -> flag1Label -> setText("True");
                ui -> flag1Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag1Label -> setText("False");
                ui -> flag1Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 2:
            if (value)
            {
                ui -> flag2Label -> setText("True");
                ui -> flag2Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag2Label -> setText("False");
                ui -> flag2Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 3:
            if (value)
            {
                ui -> flag3Label -> setText("True");
                ui -> flag3Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag3Label -> setText("False");
                ui -> flag3Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 4:
            if (value)
            {
                ui -> flag4Label -> setText("True");
                ui -> flag4Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag4Label -> setText("False");
                ui -> flag4Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 5:
            if (value)
            {
                ui -> flag5Label -> setText("True");
                ui -> flag5Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag5Label -> setText("False");
                ui -> flag5Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 6:
            if (value)
            {
                ui -> flag6Label -> setText("True");
                ui -> flag6Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag6Label -> setText("False");
                ui -> flag6Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 7:
            if (value)
            {
                ui -> flag7Label -> setText("True");
                ui -> flag7Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag7Label -> setText("False");
                ui -> flag7Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 8:
            if (value)
            {
                ui -> flag8Label -> setText("True");
                ui -> flag8Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag8Label -> setText("False");
                ui -> flag8Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 9:
            if (value)
            {
                ui -> flag9Label -> setText("True");
                ui -> flag9Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag9Label -> setText("False");
                ui -> flag9Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 10:
            if (value)
            {
                ui -> flag10Label -> setText("True");
                ui -> flag10Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag10Label -> setText("False");
                ui -> flag10Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 11:
            if (value)
            {
                ui -> flag11Label -> setText("True");
                ui -> flag11Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag11Label -> setText("False");
                ui -> flag11Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 12:
            if (value)
            {
                ui -> flag12Label -> setText("True");
                ui -> flag12Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag12Label -> setText("False");
                ui -> flag12Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 13:
            if (value)
            {
                ui -> flag13Label -> setText("True");
                ui -> flag13Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag13Label -> setText("False");
                ui -> flag13Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 14:
            if (value)
            {
                ui -> flag14Label -> setText("True");
                ui -> flag14Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag14Label -> setText("False");
                ui -> flag14Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 15:
            if (value)
            {
                ui -> flag15Label -> setText("True");
                ui -> flag15Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag15Label -> setText("False");
                ui -> flag15Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 16:
            if (value)
            {
                ui -> flag16Label -> setText("True");
                ui -> flag16Label -> setStyleSheet("QLabel { background-color : green; }");
            }
            else
            {
                ui -> flag16Label -> setText("False");
                ui -> flag16Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;

    }
}

void mainwindow::on_savePlotsButton_clicked()
{
    QPixmap pAlt = ui -> altPlot -> grab();
    pAlt.save(".\\Images\\" + QString::number(pngCounter) + " altitude.png","PNG");
    QPixmap pPrs = ui -> prsPlot -> grab();
    pPrs.save(".\\Images\\" + QString::number(pngCounter) + " pressure.png","PNG");
    QPixmap pVbat = ui -> vbatPlot -> grab();
    pVbat.save(".\\Images\\" + QString::number(pngCounter) + " battery voltage.png","PNG");
    QPixmap pT = ui -> tPlot -> grab();
    pT.save(".\\Images\\" + QString::number(pngCounter) + " temperature.png","PNG");
    QPixmap pAcc = ui -> orientPlot -> grab();
    pAcc.save(".\\Images\\" + QString::number(pngCounter) + " acceleration.png","PNG");
    ui -> gpsStack -> widget(0) -> repaint();
    QPixmap pTraj = ui -> gpsStack -> widget(0) -> grab();
    pTraj.save(".\\Images\\" + QString::number(pngCounter) + " trajectory.png","PNG");
    pngCounter++;
}

void mainwindow::on_reconnectTrcButton_clicked()
{
    transceiver -> close();
    ui -> dashTerminal -> append("STNGS \nTransceiver port closed.\n");
    portNameTrc = ui -> trcCOMEdit -> text();
    baudRateTrc = ui -> trcBaudEdit -> text();
    transceiver -> setPortName(portNameTrc);
    transceiver -> setBaudRate(baudRateTrc.toInt());
    transceiver -> open(QSerialPort::WriteOnly);
    Sleep(uint(1000));
    ui -> dashTerminal -> append("STNGS \nTransceiver port closed.\n");
}

void mainwindow::on_cmdButton_clicked()
{
    QString cmd = ui -> cmdEdit -> text();
    cmd_writeToTerminal(cmd);
}

void mainwindow::on_comboBox_2_activated(const QString &ind)
{
    if (ind == "Trajectory")
        ui -> gpsStack -> setCurrentIndex(0);
    if (ind == "Map")
        ui -> gpsStack -> setCurrentIndex(1);
}

void mainwindow::on_comboBox_3_activated(const QString &ind)
{
    if (ind == "Accelerations")
        ui -> orientStack -> setCurrentIndex(0);
    else
        ui -> orientStack -> setCurrentIndex(1);
}

void mainwindow::ws_onConnected()
{
    ws_opened = 1;
    ui -> dashTerminal -> append("WEBSOCKET \nConnected.\n");
}

void mainwindow::ws_sendMsg()
{
    int bs = 0;
    QString msg = "{\"token\" : \"" + ws_token + "\", \"vbat\": " + QString::number(vbatMain) + ", \"prs\": " + QString::number(prsMain) + ", \"t1\": " + QString::number(t1Main) + ", \"t2\" : " + QString::number(t2Main) + ", \"f\": " + QString::number(flgMain) + ", \"ax\": " + QString::number(axOrient) + ", \"ay\": " + QString::number(ayOrient) + ", \"az\": " + QString::number(azOrient) + ", \"pitch\" : " + QString::number(pitchOrient) + ", \"yaw\" : " + QString::number(yawOrient) + ", \"roll\": " + QString::number(rollOrient) + ", \"alt\" : " + QString::number(altMain) + ", \"lat\" : " + QString::number(latGPS) + ", \"lon\": " + QString::number(lonGPS) + "}";
    if (ws_opened == 1 && ws_m_collected == 1 && ws_o_collected == 1 && ws_g_collected == 1)
    {
        bs = m_webSocket.sendTextMessage(msg);
        ws_m_collected = 0; ws_o_collected = 0; ws_g_collected = 0;
        ui -> dashTerminal -> append("WEBSOCKET \nBytes send: " + QString::number(bs) + ".\n");
    }
}

void mainwindow::init_WebSocket_connection()
{
    m_webSocket.open(QUrl(QStringLiteral("ws://78.47.18.15:5000")));
    connect(&m_webSocket, &QWebSocket::connected, this, &mainwindow::ws_onConnected);
}
