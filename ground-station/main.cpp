// main.cpp
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "Counter.h"

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    Counter counter;
    // engine.rootContext()->setContextProperty("counter", &counter);
    qmlRegisterType<Counter>("Counter", 1, 0, "Counter");


    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated, &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);
    // engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    // if (engine.rootObjects().isEmpty())
    //     return -1;

    return app.exec();
}
