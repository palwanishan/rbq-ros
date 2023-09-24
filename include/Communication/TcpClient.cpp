// Copyright 2023 Rainbow Robotics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "TcpClient.h"

TcpClient::TcpClient(QObject *parent,
    const std::shared_ptr<ROBOT_STATE_DATA> &robotStateNative)
    : QObject(parent)
    , m_robotStateNative(robotStateNative)
{
    m_host = "";
    m_port = 0;

    m_timerSend = new QTimer();
    QObject::connect(m_timerSend, &QTimer::timeout, this, &TcpClient::readySend);

    m_timerReconnect = new QTimer();
    QObject::connect(m_timerReconnect, &QTimer::timeout, this, &TcpClient::reconnect);
}

TcpClient::~TcpClient()
{
    qInfo() << "Deconstructor";
}

void TcpClient::connectToHost(QString host, quint16 port)
{
    if (m_qTcpSocket->isOpen())
        disconnect();
    qInfo() << "Connecting to: " << host << " on port " << port;

    m_qTcpSocket->connectToHost(host, port);
}

void TcpClient::disconnect()
{
    m_qTcpSocket->close();
}

void TcpClient::start()
{
    m_qTcpSocket = new QTcpSocket(this);
    connect(m_qTcpSocket, &QTcpSocket::connected, this, &TcpClient::connected);
    connect(m_qTcpSocket, &QTcpSocket::disconnected, this, &TcpClient::disconnected);
    connect(m_qTcpSocket, &QTcpSocket::stateChanged, this, &TcpClient::stateChanged);
    connect(m_qTcpSocket, &QTcpSocket::readyRead, this, &TcpClient::readyRead);
    connect(m_qTcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error), this, &TcpClient::error);

    connectToHost(m_host, m_port);
    m_isAutoReconnect = true;
}

void TcpClient::connected()
{
    qInfo() << "TCP Connected!";
    m_timerSend->start(100);
    m_timerReconnect->stop();
}

void TcpClient::disconnected()
{
    qInfo() << "TCP Disconnected!";
    m_timerSend->stop();
    m_timerReconnect->start(1000);
}

void TcpClient::error(QAbstractSocket::SocketError socketError)
{
    qInfo() << "Error:" << socketError << " " << m_qTcpSocket->errorString();
}

void TcpClient::stateChanged(QAbstractSocket::SocketState socketState)
{
    QMetaEnum metaEnum = QMetaEnum::fromType<QAbstractSocket::SocketState>();
    qInfo() << "State: " << metaEnum.valueToKey(socketState);
}

void TcpClient::readyRead()
{
    // qInfo() << " -- TcpClient::readyRead() \n\t\t --> Received new packet.";

    buf.append(m_qTcpSocket->readAll());

    while(1)
    {
        bool is_header = false;
        for(int p = 0; p < buf.size()-1; p++)
        {
            if(buf[p] == (char)0xFF && buf[p+1] == (char)0xFE)
            {
                // qInfo() << " -- TcpClient::readyRead() \n\t\t --> Header is found";
                is_header = true;
                buf.remove(0, p);
                break;
            }
        }

        const int packet_size = sizeof(ROBOT_STATE_DATA)+4;
        if(is_header)
        {
            if(buf.size() >= packet_size)
            {
                // check tail
                if(buf[packet_size-2] == (char)0x00 && buf[packet_size-1] == (char)0x01)
                {
                    // qInfo() << " -- TcpClient::readyRead() \n\t\t --> Tail is found";

                    // delete header
                    buf.remove(0, 2);

                    // parising
                    QByteArray tempBuf = buf.left(sizeof(ROBOT_STATE_DATA));
                    buf.remove(0, sizeof(ROBOT_STATE_DATA)+2);

                    if(m_robotStateNative != nullptr) {
                        memcpy(m_robotStateNative.get(), tempBuf.data(), sizeof(ROBOT_STATE_DATA));
                    }

                    // qInfo() << "Received new ROBOT_STATE_DATA";
                }
                else
                {
                    // delete header only
                    buf.remove(0, 2);
                }
            }
            else
            {
                break;
            }
        }
        else
        {
            buf.clear();
            break;
        }
    }
}

quint16 TcpClient::port() const
{
    return m_port;
}

void TcpClient::setPort(quint16 newPort)
{
    m_port = newPort;
}

const QString &TcpClient::host() const
{
    return m_host;
}

void TcpClient::setHost(const QString &newHost)
{
    m_host = newHost;
}

void TcpClient::sendMsg(const QByteArray &msg) 
{
    qDebug() << "TcpClient::sendMsg(): ";
    if(m_qTcpSocket == nullptr) {
        return;
    }
    if(m_qTcpSocket->state() == QAbstractSocket::ConnectedState) {
        m_msgBufferToBeSend.append(msg);
    }
}

void TcpClient::readySend() 
{
    if(m_msgBufferToBeSend.isEmpty()) {
        return;
    }
    if(m_qTcpSocket->state() == QAbstractSocket::ConnectedState) {
        m_qTcpSocket->write(m_msgBufferToBeSend.takeFirst());
    }
}

void TcpClient::reconnect() 
{
    if(m_isAutoReconnect) {
        connectToHost(m_host, m_port);
    }
}