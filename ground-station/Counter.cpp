// counter.cpp
#include <QObject>
#include <QTimer>
#include <QtQml/qqmlregistration.h>
#include "Counter.h"

Counter::Counter(QObject *parent) : QObject(parent), m_count(0) {
    connect(&m_timer, &QTimer::timeout, this, &Counter::increaseCount);
    m_timer.start(1000); // Update every second
}

int Counter::count() const {
    return m_count;
}

void Counter::increaseCount() {
    m_count++;
    emit countChanged();
}