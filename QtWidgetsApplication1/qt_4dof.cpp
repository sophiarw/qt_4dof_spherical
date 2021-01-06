#include "qt_4dof.h"
#include "Application.h"

//------------------------------------------------------------------------------
using std::string;




qt_4dof::qt_4dof(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	// setup keyboard shortcuts
	EscKey = new QShortcut(Qt::Key_Escape, this, SLOT(ExitFullScreen()));
	FKey = new QShortcut(Qt::Key_F, this, SLOT(ToggleFullScreen()));
	SKey = new QShortcut(Qt::Key_S, this, SLOT(ToggleSettings()));
	QKey = new QShortcut(Qt::Key_Q, this, SLOT(close()));

	// sync settings state
	connect(ui.actionShow_Settings, SIGNAL(triggered(bool)), this, SLOT(ShowSettings(bool)));
	connect(ui.actionFull_Screen, SIGNAL(triggered(bool)), this, SLOT(SetFullScreen(bool)));

	// create CHAI3D application widget
	Application = new ApplicationWidget(this);
	if (Application)
	{
		Application->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		centralWidget()->layout()->addWidget(Application);
	}
	else
	{
		QMessageBox::information(this, "Application", "Cannot start application.", QMessageBox::Ok);
		close();
	}

	// configure timers
	StatusTimer = new QTimer(this);
	connect(StatusTimer, SIGNAL(timeout()), this, SLOT(UpdateStatus()));
	StatusTimer->start(1000);

	// set numeric label fonts
	QFont font("Monospace", 8);
	font.setStyleHint(QFont::TypeWriter);
	ui.labelZoom->setFont(font);

	// initialize status bar
	GraphicRate.setFrameStyle(QFrame::Panel | QFrame::Sunken);
	ui.statusBar->addPermanentWidget(&GraphicRate);
	HapticRate.setFrameStyle(QFrame::Panel | QFrame::Sunken);
	ui.statusBar->addPermanentWidget(&HapticRate);

	// set default widget configuration
	ui.sliderZoom->setValue((int)(100.0*0.05));

	//set widget values for x, y, z, theta positions
	
	Application->getNeutralPos(neutral_x, neutral_y, neutral_z, neutral_theta);
	ui.xSlider->setValue(50);
	ui.ySlider->setValue(50);
	ui.zSlider->setValue(100);
	ui.thetaSlider->setValue(50);
	ui.xPos->setText(QString("%1").arg(neutral_x, 3));
	ui.yPos->setText(QString("%1").arg(neutral_y, 3));
	ui.zPos->setText(QString("%1").arg(neutral_z, 3));
	ui.thetaPos->setText(QString("%1").arg(neutral_theta, 3));


	// show settings by default
	ShowSettings(true);

	Start();
}


//------------------------------------------------------------------------------

int qt_4dof::Start()
{
	// start haptic thread
	if (Application->start() < 0)
	{
		QMessageBox::warning(this, "CHAI3D", "No device found.", QMessageBox::Ok);
		return (-1);
	}
	// synchronize setting widgets with simulation initial state
	SyncUI();

	return (0);
}

//------------------------------------------------------------------------------

void  qt_4dof::Stop()
{
	Application->stop();
}

//------------------------------------------------------------------------------

qt_4dof::~qt_4dof()
{
	Stop();
}

//------------------------------------------------------------------------------

void  qt_4dof::SyncUI()
{
	ui.sliderZoom->setValue((int)(10.0*Application->m_demo->m_camera->getSphericalRadius()));
}

//------------------------------------------------------------------------------

void  qt_4dof::EnterFullScreen()
{
	showFullScreen();
	ui.actionFull_Screen->setChecked(true);
}

//------------------------------------------------------------------------------

void  qt_4dof::ExitFullScreen()
{
	showNormal();
	ui.actionFull_Screen->setChecked(false);
}

//------------------------------------------------------------------------------

void  qt_4dof::ToggleFullScreen()
{
	if (isFullScreen())
	{
		ExitFullScreen();
	}
	else
	{
		EnterFullScreen();
	}
}

//------------------------------------------------------------------------------

void  qt_4dof::SetFullScreen(bool fs)
{
	if (fs && !isFullScreen())
	{
		EnterFullScreen();
	}
	else if (!fs &&  isFullScreen())
	{
		ExitFullScreen();
	}
}

//------------------------------------------------------------------------------

void  qt_4dof::ToggleSettings()
{
	bool show = !ui.Settings->isVisible();
	ui.Settings->setVisible(show);
	ui.actionShow_Settings->setChecked(show);
}

//------------------------------------------------------------------------------
void  qt_4dof::ShowSettings(bool show)
{
	ui.Settings->setVisible(show);
	ui.actionShow_Settings->setChecked(show);
}

//------------------------------------------------------------------------------

void  qt_4dof::on_sliderZoom_valueChanged(int val)
{
	ui.labelZoom->setText(QString("%1").arg(val, 3));

	Application->m_demo->m_camera->setSphericalRadius((double)val / 10.0);
}

//------------------------------------------------------------------------------

void  qt_4dof::on_xSlider_valueChanged(int val)
{
	double xRange = 7.0; // simulated max is 7.8  [mm]

	//put value in appropriate range
	double x_val = val / 100.0 * 2.0*xRange - xRange;

	if (x_val < -xRange) x_val = -xRange;
	if (x_val > xRange) x_val = xRange;

	//command new x neutral position
	Application->setNeutralPosX(x_val);

	ui.xPos->setText(QString::number(x_val, 'f', 2));
}

//------------------------------------------------------------------------------

void  qt_4dof::on_ySlider_valueChanged(int val)
{
	double yRange = 9.0; // simulated max is 10  [mm]

	//put value in appropriate range
	double y_val = val / 100.0 * 2.0 * yRange - yRange;

	if (y_val < -yRange) y_val = -yRange;
	if (y_val > yRange) y_val = yRange;

	//command new x neutral position
	Application->setNeutralPosY(y_val);

	//update value sent here
	ui.yPos->setText(QString::number(y_val, 'f', 2));
}

//------------------------------------------------------------------------------

void  qt_4dof::on_zSlider_valueChanged(int val)
{
	double zRange_min = -35.0; 
	double zRange_max = -20.0; 
	double zRange = zRange_max - zRange_min;

	//put value in appropriate range
	double z_val = val / 100.0 * zRange + zRange_min;

	if (z_val < zRange_min) z_val = zRange_min;
	if (z_val > zRange_max) z_val = zRange_max;

	//command new x neutral position
	Application->setNeutralPosZ(z_val);

	//update value sent here
	ui.zPos->setText(QString::number(z_val, 'f', 2));
}

void  qt_4dof::on_thetaSlider_valueChanged(int val)
{
	double thetaRange = PI / 6;

	//put value in appropriate range
	double theta_val = val / 100.0 * 2.0 * thetaRange - thetaRange;

	if (theta_val < -thetaRange) theta_val = -thetaRange;
	if (theta_val > thetaRange) theta_val = thetaRange;

	//command new x neutral position
	Application->setNeutralPosTheta(theta_val);

	//update value sent here
	ui.thetaPos->setText(QString::number(theta_val, 'f', 2));
}


//------------------------------------------------------------------------------

void  qt_4dof::UpdateStatus()
{
	if (Application)
	{
		GraphicRate.setText(QString("graphic: %1 Hz").arg((int)(Application->getGraphicRate()), 3));
		HapticRate.setText(QString("haptic: %1 Hz").arg((int)(Application->getHapticRate()), 4));
	}
	else
	{
		GraphicRate.setText(QString("graphic: --- Hz"));
		HapticRate.setText(QString("haptic: ---- Hz"));
	}
}