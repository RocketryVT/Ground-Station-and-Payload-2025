#include "mainwindow.h"
// #include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    serial(new QSerialPort(this))
{
    ui->setupUi(this);

    // List available ports
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Port:" << info.portName() << ", Description:" << info.description();
    }

    // Set up the serial port
    serial->setPortName("COM3"); // Replace with the actual port name
    serial->setBaudRate(QSerialPort::Baud57600); // RFD900 default baud rate
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (serial->open(QIODevice::ReadOnly)) {
        connect(serial, &QSerialPort::readyRead, this, &MainWindow::readData);
        qDebug() << "Connected to" << serial->portName();
    } else {
        qDebug() << "Failed to open port:" << serial->errorString();
    }
}

MainWindow::~MainWindow()
{
    serial->close();
    delete ui;
}

void MainWindow::readData()
{
    QByteArray data = serial->readAll();
    qDebug() << "Received:" << data;
}
