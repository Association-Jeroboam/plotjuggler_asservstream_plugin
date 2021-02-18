#include "AsservStream.h"
#include <QFile>
#include <QMessageBox>
#include <thread>
#include <mutex>
#include <chrono>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions      */
#include <cstdint>
#include <QInputDialog>
#include <QStringList>
#include <QDir>
#include <QMessageBox>
#include <cmath>
#include "ui_asservstreamcontrolpanel.h"
#include "AsservStreamControlPanel.h"

#define ASSERV_FREQ 300.0
#define BAUDRATE    QSerialPort::Baud115200
using namespace PJ;


AsservStream::AsservStream():_running(false),fd(-1), fdLog(-1), deviceOpened(false)
{
    QStringList  words_list;
    words_list
        << "timestamp"

        <<"speed/right/goal"
        <<"speed/left/goal"
        <<"speed/right/current"
        <<"speed/left/current"
        <<"speed/right/error"
        <<"speed/left/error"
        ;

   std::lock_guard<std::mutex> lock( mutex() );
    foreach( const QString& name, words_list)
        dataMap().addNumeric(name.toStdString());

    controlPanelWindows = nullptr;
}


bool AsservStream::openPort()
{
    bool ok;

    QDir devDir("/dev/");
    QStringList ttyListAbsolutePath;

    QFileInfoList list = devDir.entryInfoList(QStringList() << "ttyACM*", QDir::System);
    for (int i = list.size() - 1; i >= 0; i--)
        ttyListAbsolutePath << list.at(i).absoluteFilePath();

    QString portName = QInputDialog::getItem(nullptr, tr("Which tty use ?"), tr("/dev/tty?"), ttyListAbsolutePath, 0, false, &ok);

    if (!ok)
        return false;
    device = new QSerialPort(this);
    device->setPortName(portName);
    deviceOpened = device->open(QIODevice::ReadWrite);

    if(deviceOpened){
        device->setBaudRate(BAUDRATE);
        device->setDataBits(QSerialPort::Data8);
        device->setParity(QSerialPort::NoParity);
        device->setStopBits(QSerialPort::OneStop);
        device->setFlowControl(QSerialPort::NoFlowControl);
        device->flush();
        printf("opened port %s\n", portName.toStdString().c_str());
        return true;

    } else{
        printf("Unable to open %s\n", portName.toStdString().c_str());
        return false;
    }

}

bool AsservStream::start(QStringList*)
{
    bool ok = openPort();
    bool tryAgain = !ok;
    while (tryAgain)
    {
        QMessageBox::StandardButton reply = QMessageBox::question(nullptr, "Unable to open port", "Unable to open port\nTry again?",
                QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No)
        {
            tryAgain = false;
        }
        else
        {
            ok = openPort();
            tryAgain = !ok;
        }
    }

    if (ok)
    {
        std::lock_guard < std::mutex > lock(mutex());
        // Insert dummy Point. it seems that there's a regression on plotjuggler, this dirty hack seems now necessary
        for (auto &it : dataMap().numeric)
        {
            auto &plot = it.second;

            plot.pushBack(PlotData::Point(0, 0));
        }
        _running = true;
        _thread = std::thread([this]()
        {   this->loop();});

        controlPanelWindows = new AsservStreamControlPanel(new Ui_AsservStreamControlPanel, &uartDecoder, device, fdLog);
        controlPanelWindows->show();

        return true;
    }
    else
    {
        return false;
    }
}

void AsservStream::shutdown()
{
    _running = false;

    if(controlPanelWindows != nullptr)
    	controlPanelWindows->close();

    if( _thread.joinable()){
        _thread.join();
    }
    device->clearError();
    device->close();
}

bool AsservStream::isRunning() const { return _running; }

AsservStream::~AsservStream()
{
    shutdown();
    if(controlPanelWindows != nullptr)
    	delete(controlPanelWindows);
}

void AsservStream::pushSingleCycle()
{
    UsbStreamSample sample;
    while (uartDecoder.getDecodedSample(&sample))
    {
        std::lock_guard < std::mutex > lock(mutex());

        double timestamp = double(sample.timestamp) * 1.0 / ASSERV_FREQ;

        for (auto &it : dataMap().numeric)
        {
            double value = getValueFromName(it.first, sample);

            if (std::isnan(value))
                value = 0;

            auto &plot = it.second;
            plot.pushBack(PlotData::Point(timestamp, value));
        }
    }
}

void AsservStream::loop()
{
    device->write("\r\ndata_stream start\r\n");
    device->flush();
    _running = true;
    while (_running && deviceOpened)
    {
        uint8_t read_buffer[sizeof(UsbStreamSample) + 4];
        int bytes_read = 0;

        while (device->bytesAvailable() < sizeof(UsbStreamSample) + 4){
            if (!device->waitForReadyRead(100)){
                emit closed();
            }
        }

        bytes_read = device->read((char*)read_buffer, sizeof(UsbStreamSample) + 4);

        if (bytes_read > 0)
            uartDecoder.processBytes(read_buffer, bytes_read);

        pushSingleCycle();
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

}

double AsservStream::getValueFromName(const  std::string &name, UsbStreamSample &sample)
{
    double value = 0;

    if (name == "timestamp")
        value = sample.timestamp;
    else if (name == "speed/left/current")
        value = sample.value1;
    else if (name == "speed/left/goal")
        value = sample.value2;
    else if (name == "speed/right/current")
        value = sample.value3;
    else if (name == "speed/right/goal")
        value = sample.value4;
    else if (name == "speed/left/error")
        value = sample.value2 - sample.value1;
    else if (name == "speed/right/error")
        value = sample.value4 - sample.value3;

    return value;
}



bool AsservStream::xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const 
{
    return true;
}
    
bool AsservStream::xmlLoadState(const QDomElement &parent_element)
{
    return false;
}
