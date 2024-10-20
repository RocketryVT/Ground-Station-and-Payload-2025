// counter.h
#ifndef COUNTER_H
#define COUNTER_H

#include <QObject>
#include <QTimer>
#include <QtQml/qqmlregistration.h>

class Counter : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int count READ count NOTIFY countChanged)
    QML_ELEMENT

public:
    explicit Counter(QObject *parent = nullptr);

    int count() const;

    signals:
        void countChanged();

    public slots:
        void increaseCount();

private:
    int m_count;
    QTimer m_timer;
};

#endif // COUNTER_H