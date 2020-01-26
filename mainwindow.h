#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qcustomplot.h>
#include <QSerialPort>
#include <QByteArray>

namespace Ui {
class mainwindow;
}

class mainwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit mainwindow(QWidget *parent = nullptr);
    ~mainwindow();

private:
    Ui::mainwindow *ui;
    QSerialPort *receiver;
    QSerialPort *tracker;
    QSerialPort *transceiver;
    QFrame layoutsFrame;
    int pngCounter = 1;

    QString portNameR = "COM3", baudRateR = "9600", portNameT = "COM4", baudRateT = "9600";
    QString portNameTrc = "COM5", baudRateTrc = "9600";
    QString latStation = "62.03389", lonStation = "129.73306", altStation = "85";

    int flgRaw = 0, nMain = 0, etMain = 0, vbatRaw = 0, altRawMain = 0, prsRaw = 0, t1Raw = 0, t2Raw = 0;
    int nOrient = 0, etOrient = 0, axRaw = 0, ayRaw = 0, azRaw = 0;
    int nGPS = 0, etGPS = 0, satGPS = 0, altRawGPS = 0, altGPS = 0;
    double latGPS = 0, lonGPS = 0;
    double axOrient = 0, ayOrient = 0, azOrient = 0;
    double vbatMain = 0, altMain = 0, prsMain = 0, t1Main = 0, t2Main = 0;

    QByteArray serialData;
    QString serialBuffer;
    QString parsedData;

private slots:
    void readSerial();
    void updateData(QString);
    void makePlot();
    void handleMain(QString s);
    void handleOrient(QString s);
    void handleGPS(QString s);
    void on_comboBox_currentIndexChanged(const QString &arg1);
    void on_reconnectRecButton_clicked();
    void on_trackerApplyButton_clicked();
    void on_pushButton_7_clicked();
    void writeToTracker(QString angles);
    void changeflag(int flag, bool value);
    void on_savePlotsButton_clicked();
    void on_reconnectTrcButton_clicked();
    void writeToTerminal(QString s);
    void on_cmdButton_clicked();
};

#endif // MAINWINDOW_H
