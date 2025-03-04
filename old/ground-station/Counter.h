#ifndef COUNTER_H
#define COUNTER_H

#include <QObject>
#include <QTimer>
#include <qqmlintegration.h>
#include <chrono>
#include <QDebug>

class Counter : public QObject {
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int counter READ counter WRITE setCounter NOTIFY counterChanged)

public:

    explicit Counter(QObject *parent = nullptr) : QObject(parent), m_myInt(0) {
        m_startTime = std::chrono::system_clock::now();
        connect(&m_timer, &QTimer::timeout, this, &Counter::incrementMyInt);
        m_timer.start(1000); // 1000 milliseconds = 1 second
    }

    int counter() const { return m_myInt; }

    void setCounter(int counter) {
        if (m_myInt != counter) {
            m_myInt = counter;
            emit counterChanged();
        }
    }

signals:
    void counterChanged();
    void newDataPoint(qint64 time, int value);


private slots:
    void incrementMyInt() {
        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = now - m_startTime;
        double time = elapsed_seconds.count();

        setCounter(m_myInt + 1);
        emit newDataPoint(time, m_myInt);
        qDebug() << "Emitting newDataPoint:" << time << m_myInt;
    }

private:
    int m_myInt;
    QTimer m_timer;
    std::chrono::time_point<std::chrono::system_clock> m_startTime;
};

#endif // COUNTER_H