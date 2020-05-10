#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qcustomplot.h>
#include <QSerialPort>
#include <QByteArray>
#include <QtDataVisualization>
#include <Qt3DCore>
#include <Qt3DRender>
#include <Qt3DInput>
#include <Qt3DLogic>
#include <Qt3DExtras>
#include <Qt3DAnimation>
#include <QtWebSockets/QtWebSockets>
#include <QtWebSockets/QWebSocket>
#include <QtCore/QObject>
#include <QtCore/QList>

using namespace QtDataVisualization;


namespace Ui {
class mainwindow;
}

class EchoClient : public QObject
{
    Q_OBJECT
public:
    explicit EchoClient(const QUrl &url, bool debug, QObject *parent = nullptr);
    QUrl m_url;
    bool m_debug;


Q_SIGNALS:
    void closed();


private Q_SLOTS:
    void onConnected();

};

class mainwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit mainwindow(QWidget *parent = nullptr);
    ~mainwindow();
    Q3DScatter *trajPlot = new Q3DScatter();

    QWidget *trajContainer = QWidget::createWindowContainer(trajPlot);
    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);

public slots:
    void resizeView(QSize size);

Q_SIGNALS:


private:
    Ui::mainwindow *ui;
    QWidget *container3D;
    QSerialPort *receiver;
    QSerialPort *tracker;
    QSerialPort *transceiver;
    QFrame layoutsFrame;
    int pngCounter = 1;

    QString portNameR = "COM3", baudRateR = "9600", portNameT = "COM4", baudRateT = "9600";
    QString portNameTrc = "COM5", baudRateTrc = "9600";
    QString latStation = "61.659761", lonStation = "129.388365", altStation = "85";

    int seriesIt = 0;



    QByteArray serialData;
    QString serialBuffer;
    QString parsedData;

private slots:
    void render3D();
    void readSerial();
    void updateData(QString);
    void makePlot();
    bool handleMain(QString s);
    bool handleOrient(QString s);
    bool handleGPS(QString s);
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
    void on_comboBox_2_activated(const QString &arg1);
    void updateScatter(double x, double y, double z);
    void on_comboBox_3_activated(const QString &arg1);
    void sendMsg();


protected:
    void resizeEvent(QResizeEvent *event);
};


#endif // MAINWINDOW_H
