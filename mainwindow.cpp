#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qcustomplot.h>

mainwindow::mainwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::mainwindow)
{
    ui -> setupUi(this);
    mainwindow::makePlot();

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
    QObject::connect(receiver, SIGNAL(readyRead()), this, SLOT(readSerial()));

    tracker = new QSerialPort(this);

    tracker -> setPortName(portNameT);
    tracker -> open(QSerialPort::WriteOnly);
    tracker -> setBaudRate(baudRateT.toInt());
    tracker -> setDataBits(QSerialPort::Data8);
    tracker -> setFlowControl(QSerialPort::NoFlowControl);
    tracker -> setParity(QSerialPort::NoParity);
    tracker -> setStopBits(QSerialPort::OneStop);
    tracker -> flush();
}

void mainwindow::readSerial()
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
        mainwindow::updateData(parsedData.trimmed());
    }
}

void mainwindow::updateData(QString s)
{
   ui -> dashTerminal -> append("RCVD\n" + s + "\n");
   QString type = (QString)s[8] + (QString)s[9] + (QString)s[10] + (QString)s[11] + (QString)s[12] + (QString)s[13];
   if (type == "MAIN:N")
   {
        handleMain(s);
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

        QString flagsMain = QString("%1").arg(flgRaw,0,2);
        boolean value = 0;
        for (int k = 0; k < flagsMain.length(); k++)
        {
            if (flagsMain[k] == '0')
                value = 0;
            if (flagsMain[k] == '1')
                value = 1;
            changeflag(k + 1, value);
        }

   }
   if (type == "ORIENT")
   {
        handleOrient(s);

        ui -> dashAxLabel -> setText(QString::number(axOrient) + " m/s²");
        ui -> dashAyLabel -> setText(QString::number(ayOrient) + " m/s²");
        ui -> dashAzLabel -> setText(QString::number(azOrient) + " m/s²");

        ui -> orientPlot -> graph(0) -> addData(etOrient, axOrient);
        ui -> orientPlot -> graph(1) -> addData(etOrient, ayOrient);
        ui -> orientPlot -> graph(2) -> addData(etOrient, azOrient);
        ui -> orientPlot -> rescaleAxes();
        ui -> orientPlot -> replot();
   }
   if (type == "GPS:N=")
   {
        handleGPS(s);
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
        writeToTracker("A=" + QString::number(rndAlpha) + ";" + "B=" + QString::number(rndBeta) + ";");

   }
}

void mainwindow::handleMain(QString s)
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
                            vbatRaw = temp.toInt();
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
                        altRawMain = temp.toInt();
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
                        prsRaw = temp.toInt();
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
                    t1Raw = temp.toInt();
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
                    t2Raw = temp.toInt();
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
                        flgRaw = temp.toInt();
                        temp = "";
                    }
                }
            }
        }
    }
    vbatMain = (double)vbatRaw / 100.0, altMain = (double)altRawMain / 10.0, prsMain = (double)prsRaw / 1000.0, t1Main = (double)t1Raw / 10.0, t2Main = (double)t2Raw / 10.0;
}

void mainwindow::handleOrient(QString s)
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
                    axRaw = temp.toInt();
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
                    ayRaw = temp.toInt();
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
                    azRaw = temp.toInt();
                    temp = "";
                }
            }
        }
    }
    axOrient = (double)axRaw / 10.0 * 9.81, ayOrient = (double)ayRaw / 10.0 * 9.81, azOrient = (double)azRaw / 10.0 * 9.81;
}

void mainwindow::handleGPS(QString s)
{
    // "YKTSAT5:GPS:N=12;ET=12;SAT=5;LAT=61.230;LON=129.154;ALT=200;\r\n"
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
            if (s[i - 1] == 'N') {
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
                        altRawGPS = temp.toInt();
                        temp = "";
                    }
                }
            }
        }
    }
    altGPS = (double)altRawGPS / 10.0;
}

void mainwindow::makePlot()
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

void mainwindow::writeToTracker(QString angles)
{
    angles.append( "\r");
    QByteArray ba = angles.toLatin1();
    tracker -> write(ba);
    ui -> dashTerminal -> append("SENT \n" + angles.trimmed() + "\n");
}

void mainwindow::changeflag(int flag, bool value)
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
                ui -> flag15Label -> setText("False");
                ui -> flag15Label -> setStyleSheet("QLabel { background-color : red; }");
            }
        break;
        case 15:
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
        case 16:
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

    pngCounter++;
}