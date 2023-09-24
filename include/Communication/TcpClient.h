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

#pragma once

#include <QObject>
#include <QDebug>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QMetaEnum>
#include <QNetworkProxy>
#include <QThread>
#include <QString>
#include <QTimer>

#include "../rbq_api.h"

class TcpClient : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient(QObject *parent = nullptr, 
        const std::shared_ptr<ROBOT_STATE_DATA> &robotStateNative = nullptr);
    ~TcpClient();

    const QString &host() const;
    void setHost(const QString &newHost);

    quint16 port() const;
    void setPort(quint16 newPort);

    void sendMsg(const QByteArray &msg);

public slots:
    void connectToHost(QString host, quint16 port);
    void disconnect();
    void start();
    // void setUserCommand(const USER_COMMAND &userCommand);

private slots:
    void connected();
    void disconnected();
    void error(QAbstractSocket::SocketError socketError);
    void stateChanged(QAbstractSocket::SocketState socketState);
    void readyRead();
    void readySend();
    void reconnect();

private:
    QTcpSocket *m_qTcpSocket = nullptr;
    QString m_host;
    quint16 m_port;
    QByteArray buf;
    QVector<USER_COMMAND> userCommands;
    QVector<QByteArray> m_msgBufferToBeSend;
    QTimer *m_timerSend = nullptr;
    QTimer *m_timerReconnect = nullptr;
    bool m_isAutoReconnect = false;

    std::shared_ptr<ROBOT_STATE_DATA> m_robotStateNative = nullptr;
};
