//==============================================================================
/*
\author   Sophia Williams
*/
//==============================================================================


#pragma once

#if defined(WIN32) | defined(WIN64)
#pragma warning(disable: 4100)
#endif
#include "chai3d.h"
#include <QtWidgets/QMainWindow>
#include "ui_qt_4dof.h"
//------------------------------------------------------------------------------
#include <QFileSystemModel>
#include <QOpenGLWidget>
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QShortcut>
#include <QLabel>

using std::string;
//------------------------------------------------------------------------------
class ApplicationWidget;
//------------------------------------------------------------------------------
#define PI  3.14159



namespace Ui
{
	class qt_4dofClass;
}


class qt_4dof : public QMainWindow
{
    Q_OBJECT


//--------------------------------------------------------------------------
// CONSTRUCTOR & DESTRUCTOR:
//--------------------------------------------------------------------------
public:
    qt_4dof(QWidget *parent = Q_NULLPTR);
	~qt_4dof();


//--------------------------------------------------------------------------
// PRIVATE MEMBERS - UI:
//--------------------------------------------------------------------------

private:
    Ui::qt_4dofClass ui;
	QShortcut *EscKey;
	QShortcut *FKey;
	QShortcut *SKey;
	QShortcut *QKey;
	QTimer *StatusTimer;
	ApplicationWidget *Application;
	QLabel GraphicRate;
	QLabel HapticRate;


//--------------------------------------------------------------------------
// PRIVATE MEMBERS:
//--------------------------------------------------------------------------

private:
	int AbortRequest;
	double neutral_x, neutral_y, neutral_z, neutral_theta;
	double xRange = 7.0; // simulated max is 7.8  [mm]
	double yRange = 9.0; // simulated max is 10  [mm]
	double zRange_min = -25.0;
	double zRange_max = -15.0;
	double zRange = zRange_max - zRange_min;
	double thetaRange = PI / 6;


	//--------------------------------------------------------------------------
	// PRIVATE SLOTS:
	//--------------------------------------------------------------------------

private slots:
	void on_sliderZoom_valueChanged(int val);
	void EnterFullScreen();
	void ExitFullScreen();
	void ToggleFullScreen();
	void SetFullScreen(bool fs);
	void ToggleSettings();
	void ShowSettings(bool show);
	void UpdateStatus();


	//functionality for moving x, y, z, theta positions
	void on_xSlider_valueChanged(int val);
	void on_ySlider_valueChanged(int val);
	void on_zSlider_valueChanged(int val);
	void on_thetaSlider_valueChanged(int val);


	//functionality for button presses
	void on_xMinusButton_pressed();
	void on_xPlusButton_pressed();
	void on_yMinusButton_pressed();
	void on_yPlusButton_pressed();
	void on_zMinusButton_pressed();
	void on_zPlusButton_pressed();
	void on_thetaMinusButton_pressed();
	void on_thetaPlusButton_pressed();

	

//--------------------------------------------------------------------------
// PUBLIC METHODS:
//--------------------------------------------------------------------------

public:
	int  Start();
	void Stop();
	void SyncUI();
	void updatePlot(QVector<double>& x, QVector<double>& y);

};
