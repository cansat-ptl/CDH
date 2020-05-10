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

    bool ws_opened = 0, ws_m_collected = 0, ws_g_collected = 0, ws_o_collected = 0;
    int flgMain = 0, nMain = 0, etMain = 0, vbatRaw = 0;
    int nOrient = 0, etOrient = 0;
    int nGPS = 0, etGPS = 0, satGPS = 0, altGPS = 0;
    int pitchOrient = 0, yawOrient = 0, rollOrient = 0;
    double latGPS = 0, lonGPS = 0;
    double axOrient = 0, ayOrient = 0, azOrient = 0;
    double vbatMain = 0, altMain = 0, prsMain = 0, t1Main = 0, t2Main = 0;
    QString ws_token = "b7037fe6b012ae49d6c8189b60ca2c0b3f820950b27881755d034a676d3680f5";
    QWebSocket m_webSocket;

    Qt3DCore::QTransform *transform = new Qt3DCore::QTransform;

private slots:
    void init_WebSocket_connection();
    void init_3D_render();
    void init_data_plots();
    void h_readSerial();
    void h_updateData(QString);
    bool h_handleMain(QString s);
    bool h_handleOrient(QString s);
    bool h_handleGPS(QString s);
    void on_comboBox_currentIndexChanged(const QString &arg1);
    void on_reconnectRecButton_clicked();
    void on_trackerApplyButton_clicked();
    void on_pushButton_7_clicked();
    void cmd_writeToTracker(QString angles);
    void d_changeFlag(int flag, bool value);
    void on_savePlotsButton_clicked();
    void on_reconnectTrcButton_clicked();
    void cmd_writeToTerminal(QString s);
    void on_cmdButton_clicked();
    void on_comboBox_2_activated(const QString &arg1);
    void d_updateScatter(double x, double y, double z);
    void on_comboBox_3_activated(const QString &arg1);
    void ws_sendMsg();
    void ws_onConnected();
    void td_drawLine(const QVector3D& start, const QVector3D& end, const QColor& color, Qt3DCore::QEntity *_rootEntity);


protected:
    void td_resizeEvent(QResizeEvent *event);
};


#endif // MAINWINDOW_H
