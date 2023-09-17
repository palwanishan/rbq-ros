#pragma once

#include <QObject>
#include <QDebug>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QMetaEnum>
#include <QNetworkProxy>
#include <QThread>
#include <QString>

#include "rbq_api.h"

class TcpClient : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient(QObject *parent = nullptr, 
        const std::shared_ptr<ROBOT_STATE_DATA> &robotStateNative);
    ~TcpClient();

    const QString &host() const;
    void setHost(const QString &newHost);

    quint16 port() const;
    void setPort(quint16 newPort);

signals:

public slots:
    void connectToHost(QString host, quint16 port);
    void disconnect();
    void start();

private slots:
    void connected();
    void disconnected();
    void error(QAbstractSocket::SocketError socketError);
    void stateChanged(QAbstractSocket::SocketState socketState);
    void readyRead();

private:
    QTcpSocket *m_qTcpSocket;
    QString m_host;
    quint16 m_port;
    QByteArray buf;

    std::shared_ptr<ROBOT_STATE_DATA> m_robotStateNative = nullptr;
};
