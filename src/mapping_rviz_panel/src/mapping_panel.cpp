#include "mapping_rviz_panel/mapping_panel.hpp"

#include <signal.h>
#include <unistd.h>

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QDockWidget>
#include <QFile>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QMainWindow>
#include <QMessageBox>
#include <QPalette>
#include <QProcess>
#include <QPushButton>
#include <QSaveFile>
#include <QShowEvent>
#include <QStandardPaths>
#include <QStyle>
#include <QStyleFactory>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <set>

#include <pluginlib/class_list_macros.hpp>

namespace mapping_rviz_panel
{

namespace
{

// The python3 interpreter followed by the script, so a shell whose command
// line merely mentions the script (the terminal's own bash, an editor, grep)
// never counts.
const QString kPy = QStringLiteral(R"(^[^ ]*python3[^ ]* [^ ]*)");
const QString kWorkspace = QStringLiteral("~/1Mapping");
const QString kSaveKey = QStringLiteral("save_map");
const char * const kSaveLabel = "บันทึกแผนที่";

constexpr int kRefreshMs = 1000;
constexpr qint64 kStartWaitMs = 15000;  // terminal start-up + ros2 launch import
constexpr qint64 kStopWaitMs = 5000;    // before offering a harder signal

// Widgets are picked by object name and a "state" (run, busy, ended, off) or
// "kind" (start, stop, force) property, so the confirm dialogs, which inherit
// this sheet, keep their normal look. @name@ is a colour from kTokens.
const char * const kStyle = R"(
QLabel#title { color: palette(window-text); font-size: 14px; font-weight: 600; }
QLabel#section { color: @muted@; font-size: 11px; font-weight: 600; }
QLabel#chip { border-radius: 9px; padding: 1px 8px; font-size: 11px; font-weight: 600; }
QLabel#chip[tone="run"] { color: @greenText@; background: @greenSoft@; }
QLabel#chip[tone="busy"] { color: @amberText@; background: @amberSoft@; }
QLabel#chip[tone="ended"] { color: @redText@; background: @redSoft@; }
QLabel#chip[tone="off"] { color: @muted@; background: @neutralSoft@; }
QFrame#segment { border-radius: 2px; background: @segmentOff@; }
QFrame#segment[state="run"] { background: #22c55e; }
QFrame#segment[state="busy"] { background: #f59e0b; }
QFrame#segment[state="ended"] { background: #ef4444; }
QFrame#card { background: @card@; border: 1px solid @cardBorder@; border-radius: 10px; }
QFrame#card[state="busy"] { border-color: @amberBorder@; }
QFrame#card[state="ended"] { border-color: @redBorder@; }
QLabel#badge {
  border-radius: 12px; color: white; font-size: 12px; font-weight: 700; background: #16a34a;
}
QLabel#badge[state="busy"] { background: #d97706; }
QLabel#badge[state="ended"] { background: #dc2626; }
QLabel#badge[state="off"] { color: @muted@; background: @badgeOff@; }
QLabel#name { color: palette(text); font-size: 13px; font-weight: 600; }
QLabel#status { color: @muted@; font-size: 11px; }
QLabel#status[state="run"] { color: @greenText@; }
QLabel#status[state="busy"] { color: @amberText@; }
QLabel#status[state="ended"] { color: @redText@; }
QPushButton#action {
  min-width: 72px; min-height: 28px; max-height: 28px; padding: 0 10px;
  border-radius: 14px; font-size: 12px; font-weight: 600;
}
QPushButton#action[kind="start"] { color: white; background: @greenBtn@; border: none; }
QPushButton#action[kind="start"]:hover { background: @greenBtnHover@; }
QPushButton#action[kind="stop"] {
  color: palette(text); background: transparent; border: 1px solid @stopBorder@;
}
QPushButton#action[kind="stop"]:hover {
  color: @redText@; background: @redSoft@; border-color: @redBorder@;
}
QPushButton#action[kind="force"] { color: white; background: @redBtn@; border: none; }
QPushButton#action[kind="force"]:hover { background: @redBtnHover@; }
QPushButton#action:disabled { color: @muted@; background: @neutralSoft@; border: none; }
QLabel#hint { color: @muted@; font-size: 11px; }
)";

// Colours that differ between a light and a dark RViz palette; the neutral
// ones come from the palette itself.
const struct
{
  const char * name;
  const char * light;
  const char * dark;
} kTokens[] = {
  {"card", "palette(base)", "#22262d"},
  {"cardBorder", "rgba(127, 127, 127, 0.28)", "rgba(255, 255, 255, 0.08)"},
  {"muted", "#868c96", "#8b94a1"},
  {"neutralSoft", "rgba(127, 127, 127, 0.15)", "rgba(255, 255, 255, 0.07)"},
  {"segmentOff", "rgba(127, 127, 127, 0.28)", "rgba(255, 255, 255, 0.12)"},
  {"badgeOff", "rgba(127, 127, 127, 0.18)", "rgba(255, 255, 255, 0.08)"},
  {"stopBorder", "rgba(127, 127, 127, 0.45)", "rgba(255, 255, 255, 0.18)"},
  {"greenText", "#15803d", "#4ade80"},
  {"greenSoft", "rgba(22, 163, 74, 0.13)", "rgba(74, 222, 128, 0.14)"},
  {"greenBtn", "#16a34a", "#16a34a"},
  {"greenBtnHover", "#15803d", "#22c55e"},
  {"amberText", "#b45309", "#fbbf24"},
  {"amberSoft", "rgba(217, 119, 6, 0.15)", "rgba(251, 191, 36, 0.14)"},
  {"amberBorder", "rgba(217, 119, 6, 0.6)", "rgba(245, 158, 11, 0.55)"},
  {"redText", "#dc2626", "#f87171"},
  {"redSoft", "rgba(220, 38, 38, 0.12)", "rgba(248, 113, 113, 0.14)"},
  {"redBorder", "rgba(220, 38, 38, 0.6)", "rgba(248, 113, 113, 0.5)"},
  {"redBtn", "#dc2626", "#dc2626"},
  {"redBtnHover", "#b91c1c", "#ef4444"},
};

QString panelStyle(bool dark)
{
  QString css = QString::fromUtf8(kStyle);
  for (const auto & token : kTokens) {
    css.replace(QString("@%1@").arg(token.name), dark ? token.dark : token.light);
  }
  return css;
}

// The rest of the window in the mapping RViz: menus, tool bar, status bar,
// scroll bars and plain buttons. Inputs and trees are left to Fusion and the
// palette, which draw them well and keep their arrows and check marks.
const char * const kWindowStyle = R"(
QToolTip { color: #e3e6ea; background: #2a2f37; border: 1px solid #3a404a; padding: 4px 6px; }
QMenuBar { background: #15171b; color: #c9ced6; }
QMenuBar::item { padding: 4px 10px; background: transparent; }
QMenuBar::item:selected { background: #262a31; border-radius: 4px; }
QMenu { background: #1f2329; color: #e3e6ea; border: 1px solid #30353d; padding: 4px; }
QMenu::item { padding: 5px 22px 5px 12px; border-radius: 4px; }
QMenu::item:selected { background: #2f6fde; color: white; }
QMenu::separator { height: 1px; background: #30353d; margin: 4px 6px; }
QToolBar { background: #15171b; border: none; border-bottom: 1px solid #262a31; padding: 4px; spacing: 4px; }
QToolBar QToolButton { color: #c9ced6; padding: 5px 10px; border-radius: 6px; }
QToolBar QToolButton:hover { background: #262a31; color: white; }
QToolBar QToolButton:checked { background: #2f6fde; color: white; }
QStatusBar { background: #15171b; color: #8b94a1; border-top: 1px solid #262a31; }
QStatusBar QLabel { color: #8b94a1; }
QHeaderView::section {
  background: #1f2329; color: #8b94a1; border: none; border-right: 1px solid #262a31; padding: 4px 6px;
}
QScrollBar:vertical { background: transparent; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #3a404a; border-radius: 4px; min-height: 28px; margin: 2px; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 0; }
QScrollBar::handle:horizontal { background: #3a404a; border-radius: 4px; min-width: 28px; margin: 2px; }
QScrollBar::handle:hover { background: #4a515c; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }
QMainWindow::separator { background: #121418; width: 4px; height: 4px; }
QTabBar::tab { background: #15171b; color: #8b94a1; padding: 6px 14px; border: none; }
QTabBar::tab:selected { color: #e3e6ea; background: #1b1e23; border-bottom: 2px solid #2f6fde; }
QPushButton {
  background: #262a31; color: #e3e6ea; border: 1px solid #343a43; border-radius: 6px; padding: 4px 12px;
}
QPushButton:hover { background: #2e333b; }
QPushButton:disabled { color: #6b7380; }
)";

// Dark Fusion for the whole RViz process, matching the dark 3D view. Only when
// mapping_panel.launch.py asks for it (MAPPING_RVIZ_THEME=dark): added to any
// other RViz, the panel leaves its look alone and follows its palette.
void applyDarkTheme()
{
  static bool applied = false;
  if (applied) {
    return;
  }
  applied = true;
  QApplication::setStyle(QStyleFactory::create("Fusion"));
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(27, 30, 35));
  palette.setColor(QPalette::WindowText, QColor(227, 230, 234));
  palette.setColor(QPalette::Base, QColor(20, 22, 26));
  palette.setColor(QPalette::AlternateBase, QColor(26, 29, 34));
  palette.setColor(QPalette::Text, QColor(227, 230, 234));
  palette.setColor(QPalette::Button, QColor(38, 42, 49));
  palette.setColor(QPalette::ButtonText, QColor(227, 230, 234));
  palette.setColor(QPalette::BrightText, Qt::white);
  palette.setColor(QPalette::Highlight, QColor(47, 111, 222));
  palette.setColor(QPalette::HighlightedText, Qt::white);
  palette.setColor(QPalette::Link, QColor(110, 168, 254));
  palette.setColor(QPalette::ToolTipBase, QColor(42, 47, 55));
  palette.setColor(QPalette::ToolTipText, QColor(227, 230, 234));
  palette.setColor(QPalette::PlaceholderText, QColor(107, 115, 128));
  palette.setColor(QPalette::Light, QColor(58, 64, 74));
  palette.setColor(QPalette::Midlight, QColor(48, 53, 61));
  palette.setColor(QPalette::Mid, QColor(38, 42, 49));
  palette.setColor(QPalette::Dark, QColor(18, 20, 24));
  palette.setColor(QPalette::Shadow, QColor(11, 12, 14));
  for (const QPalette::ColorRole role : {QPalette::WindowText, QPalette::Text, QPalette::ButtonText}) {
    palette.setColor(QPalette::Disabled, role, QColor(107, 115, 128));
  }
  QApplication::setPalette(palette);
  qApp->setStyleSheet(kWindowStyle);
}

bool matches(const QRegularExpression & re, const QString & cmd)
{
  return !re.pattern().isEmpty() && re.match(cmd).hasMatch();
}

// Sets a property the style sheet selects on, re-polishing only on a change.
void restyle(QWidget * widget, const char * name, const QString & value)
{
  if (widget->property(name).toString() == value) {
    return;
  }
  widget->setProperty(name, value);
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
}

// Seconds since the process started, or -1 if it is gone.
qint64 secondsRunning(pid_t pid)
{
  QFile stat_file(QStringLiteral("/proc/%1/stat").arg(pid));
  QFile uptime_file(QStringLiteral("/proc/uptime"));
  if (!stat_file.open(QIODevice::ReadOnly) || !uptime_file.open(QIODevice::ReadOnly)) {
    return -1;
  }
  // Fields counted from the one after the ")" closing the command name, which
  // may contain spaces: index 19 is field 22, the start time in clock ticks.
  const QByteArray stat = stat_file.readAll();
  const QList<QByteArray> fields = stat.mid(stat.lastIndexOf(')') + 2).split(' ');
  if (fields.size() < 20) {
    return -1;
  }
  const double uptime = uptime_file.readAll().split(' ').value(0).toDouble();
  return static_cast<qint64>(uptime - fields[19].toDouble() / sysconf(_SC_CLK_TCK));
}

// When the process started, or an invalid time if it is gone.
QDateTime startedAt(pid_t pid)
{
  const qint64 seconds = secondsRunning(pid);
  return seconds < 0 ? QDateTime() : QDateTime::currentDateTime().addSecs(-seconds);
}

QString formatUptime(qint64 seconds)
{
  if (seconds < 60) {
    return "เพิ่งเริ่ม";
  }
  const qint64 minutes = seconds / 60;
  if (minutes < 60) {
    return QString("%1 นาที").arg(minutes);
  }
  return QString("%1 ชม. %2 นาที").arg(minutes / 60).arg(minutes % 60);
}

// "16:42" for today, "07/10 23:36" for another day.
QString formatWhen(const QDateTime & when)
{
  return when.toString(when.date() == QDate::currentDate() ? "HH:mm" : "dd/MM HH:mm");
}

// "16:42 · 233 MB"
QString describe(const QFileInfo & file)
{
  return formatWhen(file.lastModified()) + " · " +
         QLocale::c().formattedDataSize(file.size(), 0, QLocale::DataSizeSIFormat);
}

// Where the save button writes: the destination of the usual save_map call.
QString saveDir()
{
  return QDir::homePath() + "/Downloads/LOAM";
}

QString tilde(QString path)
{
  return path.startsWith(QDir::homePath()) ? path.replace(0, QDir::homePath().size(), "~") : path;
}

// The folder LIO-SAM deletes and rewrites by itself when it shuts down
// (savePCD in the params.yaml run.launch.py loads), or empty when that is off.
QString exitSaveDir()
{
  QFile file(QDir::homePath() + "/1Mapping/install/lio_sam/share/lio_sam/config/params.yaml");
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return QString();
  }
  const QString text = QString::fromUtf8(file.readAll());
  const QRegularExpression on(
    R"(^\s*savePCD:\s*true\b)", QRegularExpression::MultilineOption);
  const QRegularExpression directory(
    R"re(^\s*savePCDDirectory:\s*"([^"]*)")re", QRegularExpression::MultilineOption);
  if (!on.match(text).hasMatch()) {
    return QString();
  }
  const QRegularExpressionMatch match = directory.match(text);
  return QDir::cleanPath(
    QDir::homePath() + (match.hasMatch() ? match.captured(1) : "/Downloads/LOAM/"));
}

}  // namespace

MappingPanel::MappingPanel(QWidget * parent)
: rviz_common::Panel(parent),
  script_dir_(QStandardPaths::writableLocation(QStandardPaths::RuntimeLocation) +
    QStringLiteral("/mapping_panel"))
{
  addStep(
    "ouster", "LiDAR Ouster",
    "ros2 launch ouster_ros driver.launch.py",
    kPy + R"(ros2 launch ouster_ros driver\.launch\.py)");
  // run.launch.py without its RViz: the map shows in the RViz this panel is
  // in. Started by hand with run.launch.py itself, it counts just the same.
  addStep(
    "lio_sam", "LIO-SAM",
    "ros2 launch mapping_rviz_panel lio_sam.launch.py",
    kPy + R"(ros2 launch (lio_sam run|mapping_rviz_panel lio_sam)\.launch\.py)");
  // The start_cart.sh of 2Localize_get_path, not the one in this workspace:
  // it resets can0 and waits for the STM32 (sudo password and the power-cycle
  // prompt appear in its terminal), then execs the low_level launch and
  // nothing else. ~/1Mapping/start_cart.sh is the whole driving stack: it
  // brings up its own LiDAR, GPS and NDT localization next to steps 1, 4 and
  // 5, and its CAN stage ends in "Device doesn't support restart from Bus
  // Off" because it calls the 2Localize copy of setup_can0.sh.
  addStep(
    "lowlevel", "Low-level",
    "bash ~/2Localize_get_path/start_cart.sh",
    kPy + R"(ros2 launch golfcart_low_level )",
    R"(^[^ ]*bash [^ ]*start_cart\.sh)");
  addStep(
    "gps", "GPS u-blox",
    "ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch.py",
    kPy + R"(ros2 launch ublox_gps ublox_gps_node_zed_f9r-launch\.py)");
  addStep(
    "ntrip", "NTRIP (RTK)",
    "ros2 launch ntrip_client ntrip_client-launch.py",
    kPy + R"(ros2 launch ntrip_client ntrip_client-launch\.py)");

  find("lio_sam")->confirm_stop =
    "หยุด LIO-SAM?\n\n"
    "แผนที่ที่ทำอยู่มีแค่ในหน่วยความจำ หยุดแล้วต้องเริ่มทำใหม่ตั้งแต่ต้น";
  Step * lowlevel = find("lowlevel");
  // The STM32 firmware only raises a flag when /speed_cmd stops arriving; it
  // keeps driving at the last target speed.
  lowlevel->confirm_stop =
    "หยุด low_level?\n\n"
    "ถ้ารถยังวิ่งอยู่ STM32 จะวิ่งต่อด้วยความเร็วสุดท้าย "
    "เพราะไม่หยุดเองเมื่อคำสั่งขาด ให้จอดรถให้นิ่งก่อน\n\n"
    "จะเปิดใหม่ให้ใช้ปุ่มนี้ (start_cart.sh) เสมอ";
  // What start_cart.sh is blocked on, told by the command it is running.
  lowlevel->waits = {
    {QRegularExpression("^sudo "), "รอรหัส sudo ใน terminal"},
    {QRegularExpression("^(timeout [^ ]+ )?candump "), "รอสัญญาณ STM32 · ดู terminal"},
  };
  // Without it the launch waits 10 s for the receiver and then dies.
  find("gps")->device = "/dev/ttyACM0";

  save_.command = QString(
    "ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "
    "\"{resolution: 0.2, destination: '%1'}\"").arg(saveDir());
  save_.node.setPattern(R"(^[^ ]*/lio_sam_mapOptimization( |$))");
  save_.calling.setPattern(kPy + R"(ros2 service call /lio_sam/save_map )");
  save_.terminal.setPattern(
    "^[^ ]*bash .*" + QRegularExpression::escape(script_dir_ + "/" + kSaveKey + ".sh") + "$");

  themed_ = qgetenv("MAPPING_RVIZ_THEME") == "dark";
  if (themed_) {
    applyDarkTheme();
  }
  buildUi();

  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &MappingPanel::refresh);
  timer->start(kRefreshMs);
  refresh();
}

void MappingPanel::addStep(
  const QString & key, const QString & name, const QString & command,
  const QString & running, const QString & starting)
{
  Step step;
  step.key = key;
  step.number = static_cast<int>(steps_.size()) + 1;
  step.name = name;
  step.label = QString("%1. %2").arg(step.number).arg(name);
  step.command = command;
  step.running.setPattern(running);
  step.starting.setPattern(starting);
  step.terminal.setPattern(
    "^[^ ]*bash .*" + QRegularExpression::escape(script_dir_ + "/" + key + ".sh") + "$");
  steps_.push_back(step);
}

// No scroll area on purpose: the dock then cannot get shorter than the steps.
void MappingPanel::buildUi()
{
  setStyleSheet(panelStyle(palette().color(QPalette::Window).lightness() < 128));

  auto * title = new QLabel("ระบบทำแผนที่");
  title->setObjectName("title");
  running_chip_ = new QLabel;
  busy_chip_ = new QLabel;
  ended_chip_ = new QLabel;
  auto * header = new QHBoxLayout;
  header->setSpacing(6);
  header->addWidget(title);
  header->addStretch();
  for (QLabel * chip : {busy_chip_, ended_chip_, running_chip_}) {
    chip->setObjectName("chip");
    header->addWidget(chip);
  }
  restyle(busy_chip_, "tone", "busy");
  restyle(ended_chip_, "tone", "ended");

  // One segment per step, in start-up order: the whole chain at a glance.
  auto * strip = new QHBoxLayout;
  strip->setSpacing(3);
  for (Step & step : steps_) {
    step.segment = new QFrame;
    step.segment->setObjectName("segment");
    step.segment->setFixedHeight(4);
    strip->addWidget(step.segment);
  }

  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(5);
  layout->addLayout(header);
  layout->addLayout(strip);
  layout->addSpacing(2);
  for (size_t i = 0; i < steps_.size(); ++i) {
    layout->addWidget(buildCard(steps_[i], i));
  }
  auto * section = new QLabel("แผนที่ · " + tilde(saveDir()));
  section->setObjectName("section");
  layout->addSpacing(4);
  layout->addWidget(section);
  layout->addWidget(buildSaveCard());

  auto * hint = new QLabel("หยุด = Ctrl+C ในหน้าต่างนั้น · ปิด RViz แล้วทุกขั้นยังรันต่อ");
  hint->setObjectName("hint");
  hint->setWordWrap(true);
  layout->addWidget(hint);
  layout->addStretch();
  setLayout(layout);
}

QFrame * MappingPanel::buildCard(Step & step, size_t index)
{
  step.card = new QFrame;
  step.card->setObjectName("card");
  step.card->setToolTip(step.command);

  step.badge = new QLabel(QString::number(step.number));
  step.badge->setObjectName("badge");
  step.badge->setAlignment(Qt::AlignCenter);
  step.badge->setFixedSize(24, 24);

  auto * name = new QLabel(step.name);
  name->setObjectName("name");
  step.status = new QLabel;  // one line, so the dock widens rather than clips it
  step.status->setObjectName("status");
  auto * texts = new QVBoxLayout;
  texts->setSpacing(0);
  texts->addWidget(name);
  texts->addWidget(step.status);

  step.button = new QPushButton;
  step.button->setObjectName("action");
  step.button->setCursor(Qt::PointingHandCursor);
  connect(step.button, &QPushButton::clicked, this, [this, index]() {onClicked(index);});

  auto * row = new QHBoxLayout(step.card);
  row->setContentsMargins(8, 5, 8, 5);
  row->setSpacing(10);
  row->addWidget(step.badge);
  row->addLayout(texts, 1);
  row->addWidget(step.button);
  return step.card;
}

QFrame * MappingPanel::buildSaveCard()
{
  save_.card = new QFrame;
  save_.card->setObjectName("card");
  save_.card->setToolTip(save_.command);

  save_.badge = new QLabel("↓");
  save_.badge->setObjectName("badge");
  save_.badge->setAlignment(Qt::AlignCenter);
  save_.badge->setFixedSize(24, 24);

  auto * name = new QLabel(kSaveLabel);
  name->setObjectName("name");
  save_.status = new QLabel;
  save_.status->setObjectName("status");
  auto * texts = new QVBoxLayout;
  texts->setSpacing(0);
  texts->addWidget(name);
  texts->addWidget(save_.status);

  save_.button = new QPushButton;
  save_.button->setObjectName("action");
  save_.button->setCursor(Qt::PointingHandCursor);
  connect(save_.button, &QPushButton::clicked, this, &MappingPanel::onSaveClicked);

  auto * row = new QHBoxLayout(save_.card);
  row->setContentsMargins(8, 5, 8, 5);
  row->setSpacing(10);
  row->addWidget(save_.badge);
  row->addLayout(texts, 1);
  row->addWidget(save_.button);
  return save_.card;
}

MappingPanel::Step * MappingPanel::find(const QString & key)
{
  for (Step & step : steps_) {
    if (step.key == key) {
      return &step;
    }
  }
  return nullptr;
}

std::vector<MappingPanel::Proc> MappingPanel::listProcs()
{
  std::vector<Proc> procs;
  const QStringList entries =
    QDir(QStringLiteral("/proc")).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
  for (const QString & name : entries) {
    bool ok = false;
    const pid_t pid = name.toInt(&ok);
    if (!ok) {
      continue;
    }
    QFile file("/proc/" + name + "/cmdline");
    if (!file.open(QIODevice::ReadOnly)) {
      continue;
    }
    QByteArray raw = file.readAll();
    const pid_t pgid = getpgid(pid);
    if (raw.isEmpty() || pgid < 0) {
      continue;  // kernel thread, or gone meanwhile
    }
    if (raw.endsWith('\0')) {
      raw.chop(1);
    }
    raw.replace('\0', ' ');
    procs.push_back({pid, pgid, QString::fromLocal8Bit(raw)});
  }
  return procs;
}

bool MappingPanel::isUp(const Step & step, const std::vector<Proc> & procs) const
{
  return std::any_of(
    procs.begin(), procs.end(), [&step](const Proc & proc) {
      return matches(step.running, proc.cmd) || matches(step.starting, proc.cmd);
    });
}

// Signal the whole process group, which is exactly what Ctrl+C in its terminal
// does, only when the group belongs to this step: led by this step's terminal
// script, or by the step itself when it was started by hand. A LiDAR driver
// that ~/1Mapping/start_cart.sh, run by hand, brought up inside its own group
// gets only its own signal.
bool MappingPanel::ownsGroup(
  const Step & step, const Proc & proc, const std::vector<Proc> & procs) const
{
  if (proc.pgid <= 1 || proc.pgid == getpgrp()) {
    return false;
  }
  for (const Proc & leader : procs) {
    if (leader.pid == proc.pgid) {
      return matches(step.terminal, leader.cmd) || matches(step.running, leader.cmd) ||
             matches(step.starting, leader.cmd);
    }
  }
  return false;
}

// The other step whose process group this one runs in, if any.
const MappingPanel::Step * MappingPanel::ownerOf(
  const Step & step, const Proc & proc, const std::vector<Proc> & procs) const
{
  for (const Proc & leader : procs) {
    if (leader.pid != proc.pgid) {
      continue;
    }
    for (const Step & other : steps_) {
      if (&other != &step &&
        (matches(other.terminal, leader.cmd) || matches(other.running, leader.cmd) ||
        matches(other.starting, leader.cmd)))
      {
        return &other;
      }
    }
  }
  return nullptr;
}

MappingPanel::MapState MappingPanel::mapState(const std::vector<Proc> & procs) const
{
  MapState map;
  map.file = QFileInfo(saveDir() + "/GlobalMap.pcd");
  QDateTime since;
  for (const Proc & proc : procs) {
    map.saving = map.saving || matches(save_.calling, proc.cmd);
    if (!map.mapping && matches(save_.node, proc.cmd)) {
      map.mapping = true;
      since = startedAt(proc.pid);
    }
  }
  map.saved = since.isValid() && map.file.exists() && map.file.lastModified() >= since;
  return map;
}

// For LIO-SAM the question depends on what is on disk: the map it has built
// exists nowhere else until it is saved.
QString MappingPanel::stopQuestion(const Step & step, const std::vector<Proc> & procs) const
{
  QString text = step.confirm_stop;
  if (step.key != "lio_sam") {
    return text;
  }
  const MapState map = mapState(procs);
  if (map.saving) {
    text += "\n\n⚠ กำลังบันทึกแผนที่อยู่ — รอให้เสร็จก่อน";
  } else if (!map.saved) {
    text += "\n\n⚠ รอบนี้ยังไม่ได้บันทึกแผนที่";
  } else {
    text += "\n\nบันทึกล่าสุด " + formatWhen(map.file.lastModified()) +
      " — ที่วิ่งหลังจากนั้นไม่อยู่ในไฟล์";
  }
  // With savePCD on, mapOptimization answers Ctrl+C by deleting this folder
  // and writing cloudGlobal.pcd and friends in its place.
  if (map.file.exists() && exitSaveDir() == QDir::cleanPath(saveDir())) {
    text += "\n\n⚠ params.yaml ตั้ง savePCD: true ไว้ — ตอนปิด LIO-SAM จะลบ " +
      tilde(saveDir()) + " แล้วเซฟใหม่เอง แผนที่ที่บันทึกไว้ในนั้นจะหาย "
      "ให้คัดลอกออกไปก่อนกดหยุด";
  }
  return text;
}

void MappingPanel::onClicked(size_t index)
{
  Step & step = steps_[index];
  const std::vector<Proc> procs = listProcs();
  const bool stopping = step.stop_level > 0 || isUp(step, procs);

  if (stopping) {
    if (step.stop_level == 0 && !step.confirm_stop.isEmpty() &&
      QMessageBox::warning(
        this, step.label, stopQuestion(step, procs), QMessageBox::Yes | QMessageBox::Cancel,
        QMessageBox::Cancel) != QMessageBox::Yes)
    {
      return;
    }
    stop(step, listProcs());
  } else {
    if (!step.device.isEmpty() && !QFile::exists(step.device) &&
      QMessageBox::warning(
        this, step.label,
        "ไม่พบ " + step.device + "\n"
        "เครื่องยังไม่เห็นอุปกรณ์ เช็กสาย USB ก่อน\n\n"
        "เปิดต่อไหม? (launch จะรออุปกรณ์อีก 10 วินาที)",
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
    {
      return;
    }
    if (run(step.key, step.label, step.command, step.terminal, listProcs())) {
      step.start_clock.start();
    }
  }
  refresh();
}

void MappingPanel::onSaveClicked()
{
  const MapState map = mapState(listProcs());
  if (!map.mapping || map.saving) {
    return;
  }
  // The service deletes the folder before it writes, so the first save of a
  // run replaces the map of the run before. Later ones replace an older copy
  // of the same map.
  if (map.file.exists() && !map.saved &&
    QMessageBox::warning(
      this, kSaveLabel,
      "บันทึกแผนที่ลง " + tilde(saveDir()) + "?\n\n"
      "ของเดิมในโฟลเดอร์นี้จะถูกลบก่อน (GlobalMap.pcd " + describe(map.file) + ")\n"
      "ถ้ายังต้องใช้ ให้คัดลอกออกไปก่อน\n\n"
      "ใช้เวลาสักพัก ระหว่างนั้นอย่าปิด LIO-SAM",
      QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
  {
    return;
  }
  if (run(kSaveKey, kSaveLabel, save_.command, save_.terminal, listProcs())) {
    save_.start_clock.start();
  }
  refresh();
}

// Opens a terminal on the command; false, after saying why, if it could not.
bool MappingPanel::run(
  const QString & key, const QString & label, const QString & command,
  const QRegularExpression & terminal, const std::vector<Proc> & procs)
{
  // A terminal left from a run that ended by itself only waits for Enter;
  // close it so the new run does not open next to it.
  for (const Proc & proc : procs) {
    if (matches(terminal, proc.cmd)) {
      ::kill(proc.pid, SIGHUP);
    }
  }
  const QString script = writeScript(key, label, command);
  if (script.isEmpty()) {
    QMessageBox::warning(this, label, "เขียนสคริปต์ไม่ได้: " + script_dir_);
    return false;
  }
  if (!openTerminal(label, script)) {
    QMessageBox::warning(
      this, label, "เปิด terminal ไม่ได้ (ไม่พบ terminator หรือ gnome-terminal)");
    return false;
  }
  return true;
}

void MappingPanel::stop(Step & step, const std::vector<Proc> & procs)
{
  // Ctrl+C first, as in the terminal; TERM, then KILL, if pressed again.
  static const int kSignals[] = {SIGINT, SIGTERM, SIGKILL};
  const int sig = kSignals[std::min(step.stop_level, 2)];
  std::set<pid_t> groups;
  for (const Proc & proc : procs) {
    if (!matches(step.running, proc.cmd) && !matches(step.starting, proc.cmd)) {
      continue;
    }
    if (ownsGroup(step, proc, procs)) {
      if (groups.insert(proc.pgid).second) {
        ::kill(-proc.pgid, sig);
      }
    } else {
      ::kill(proc.pid, sig);
    }
  }
  step.stop_level = std::min(step.stop_level + 1, 3);
  step.stop_clock.start();
  step.start_clock.invalidate();
}

// The command goes in a script because terminator joins everything after -x
// into one string, which breaks any quoting; the terminal only gets
// "bash <script>". QSaveFile renames over the old copy, so a terminal still
// reading it keeps the old file.
QString MappingPanel::writeScript(
  const QString & key, const QString & label, const QString & command) const
{
  if (!QDir().mkpath(script_dir_)) {
    return QString();
  }
  const QString path = script_dir_ + "/" + key + ".sh";
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return QString();
  }
  const QString text =
    "# เขียนโดยปุ่ม \"" + label + "\" ใน RViz (mapping_rviz_panel) ทุกครั้งที่กดเปิด\n"
    "cd " + kWorkspace + " && source install/setup.bash && " + command + "\n"
    "code=$?\n"
    "echo\n"
    "echo \"== " + label + " จบแล้ว (exit $code) — กด Enter เพื่อปิดหน้าต่างนี้\"\n"
    "read -r\n";
  file.write(text.toUtf8());
  return file.commit() ? path : QString();
}

bool MappingPanel::openTerminal(const QString & title, const QString & script) const
{
  QProcess process;
  if (!QStandardPaths::findExecutable("terminator").isEmpty()) {
    process.setProgram("terminator");
    process.setArguments({"-T", title, "-x", "bash", script});
  } else if (!QStandardPaths::findExecutable("gnome-terminal").isEmpty()) {
    process.setProgram("gnome-terminal");
    process.setArguments({"--title", title, "--", "bash", script});
  } else {
    process.setProgram("x-terminal-emulator");
    process.setArguments({"-T", title, "-e", "bash", script});
  }
  // The step itself talks to the terminal's own tty. Without this the
  // terminal program would keep RViz's stdout and stderr open (a pipe, when
  // RViz runs under ros2 launch) for as long as any window stays open.
  process.setStandardInputFile(QProcess::nullDevice());
  process.setStandardOutputFile(QProcess::nullDevice());
  process.setStandardErrorFile(QProcess::nullDevice());
  return process.startDetached();
}

void MappingPanel::refresh()
{
  const std::vector<Proc> procs = listProcs();
  int running_count = 0;
  int busy_count = 0;
  int ended_count = 0;
  for (Step & step : steps_) {
    const Proc * running = nullptr;
    const Proc * preparing = nullptr;
    bool terminal_open = false;
    for (const Proc & proc : procs) {
      if (!running && matches(step.running, proc.cmd)) {
        running = &proc;
      }
      if (!preparing && matches(step.starting, proc.cmd)) {
        preparing = &proc;
      }
      terminal_open = terminal_open || matches(step.terminal, proc.cmd);
    }
    const bool up = running || preparing;

    if (up || (step.start_clock.isValid() && step.start_clock.elapsed() > kStartWaitMs)) {
      step.start_clock.invalidate();
    }
    if (!up && step.stop_level > 0) {
      // Stopped from here: close its terminal as well, like closing the window
      // after Ctrl+C. One that ended by itself stays open so it can be read.
      for (const Proc & proc : procs) {
        if (matches(step.terminal, proc.cmd)) {
          ::kill(proc.pid, SIGHUP);
        }
      }
      step.stop_level = 0;
      terminal_open = false;
    }

    QString state;   // run, busy, ended or off
    QString text;
    QString tip;
    QString action;
    QString kind;    // start, stop or force; empty leaves the button disabled
    if (step.stop_level > 0) {
      state = "busy";
      const bool can_force = step.stop_clock.elapsed() >= kStopWaitMs;
      text = can_force ? "ยังไม่หยุด · กดบังคับหยุดได้" : "กำลังหยุด…";
      tip = "ส่ง Ctrl+C ไปแล้ว ถ้าผ่านไป 5 วินาทียังไม่หยุด กดอีกครั้งเพื่อบังคับหยุด";
      action = !can_force ? "รอ…" : step.stop_level == 1 ? "บังคับหยุด" : "ฆ่าทิ้ง";
      kind = can_force ? "force" : "";
    } else if (running) {
      state = "run";
      const Step * owner = ownerOf(step, *running, procs);
      if (owner) {
        text = QString("รันอยู่ · มากับขั้นที่ %1").arg(owner->number);
        tip = "รันอยู่ในกลุ่มของ " + owner->label + " — หยุดขั้นนั้นแล้วตัวนี้หยุดด้วย";
      } else {
        const qint64 seconds = secondsRunning(running->pid);
        text = seconds < 0 ? "รันอยู่" : "รันอยู่ · " + formatUptime(seconds);
      }
      action = "■ หยุด";
      kind = "stop";
    } else if (preparing) {
      state = "busy";
      text = "กำลังเตรียม · ดู terminal";
      for (const Proc & proc : procs) {
        for (const Wait & wait : step.waits) {
          if (proc.pgid == preparing->pgid && matches(wait.process, proc.cmd)) {
            text = wait.text;
          }
        }
      }
      tip = "start_cart.sh กำลังรีเซ็ต can0 และรอ STM32 ก่อนเปิด low_level";
      action = "■ หยุด";
      kind = "stop";
    } else if (step.start_clock.isValid()) {
      state = "busy";
      text = "กำลังเปิด…";
      action = "รอ…";
    } else if (terminal_open) {
      state = "ended";
      text = "จบเอง · ดูสาเหตุใน terminal";
      tip = "หน้าต่าง terminal ของขั้นนี้ยังเปิดค้างไว้ให้อ่านข้อความก่อนจบ";
      action = "↻ เปิดใหม่";
      kind = "start";
    } else {
      state = "off";
      text = "ปิดอยู่";
      action = "▶ เปิด";
      kind = "start";
    }
    if ((state == "off" || state == "ended") && !step.device.isEmpty() &&
      !QFile::exists(step.device))
    {
      state = "busy";
      text = "ไม่พบ " + step.device + " · เช็กสาย USB";
      tip = "เครื่องยังไม่เห็นอุปกรณ์ (ดูได้จาก lsusb) เสียบสาย USB ให้แน่น "
        "หรือลองสายหรือพอร์ตอื่น แล้วรอให้ " + step.device + " ขึ้นก่อนกดเปิด";
    }
    running_count += state == "run";
    busy_count += state == "busy";
    ended_count += state == "ended";

    step.status->setText(text);
    step.status->setToolTip(tip);
    step.segment->setToolTip(step.label + " · " + text);
    step.button->setText(action);
    step.button->setEnabled(!kind.isEmpty());
    restyle(step.button, "kind", kind);
    for (QWidget * widget :
      std::initializer_list<QWidget *>{step.card, step.badge, step.status, step.segment})
    {
      restyle(widget, "state", state);
    }
  }

  running_chip_->setText(
    QString("รันอยู่ %1/%2").arg(running_count).arg(static_cast<int>(steps_.size())));
  restyle(running_chip_, "tone", running_count > 0 ? "run" : "off");
  busy_chip_->setText(QString("รอ %1").arg(busy_count));
  busy_chip_->setVisible(busy_count > 0);
  ended_chip_->setText(QString("จบเอง %1").arg(ended_count));
  ended_chip_->setVisible(ended_count > 0);
  refreshSave(procs);
}

void MappingPanel::refreshSave(const std::vector<Proc> & procs)
{
  const MapState map = mapState(procs);
  const Proc * terminal = nullptr;
  for (const Proc & proc : procs) {
    if (matches(save_.terminal, proc.cmd)) {
      terminal = &proc;
      break;
    }
  }
  if (map.saving ||
    (save_.start_clock.isValid() && save_.start_clock.elapsed() > kStartWaitMs))
  {
    save_.start_clock.invalidate();
  }
  // A save that worked leaves nothing to read: the map is newer than the
  // terminal that asked for it, so close that. One that failed stays open.
  if (terminal && !map.saving && map.file.exists()) {
    const QDateTime opened = startedAt(terminal->pid);
    if (opened.isValid() && map.file.lastModified() >= opened) {
      ::kill(terminal->pid, SIGHUP);
      terminal = nullptr;
      save_.start_clock.invalidate();
    }
  }

  QString state;
  QString text;
  QString tip;
  QString action = "↓ บันทึก";
  bool enabled = false;
  if (map.saving) {
    state = "busy";
    text = "กำลังบันทึก… อย่าปิด LIO-SAM";
    tip = "LIO-SAM กำลังกรองและเขียนไฟล์ แผนที่ใหญ่ใช้เวลาเป็นนาที";
    action = "รอ…";
  } else if (save_.start_clock.isValid()) {
    state = "busy";
    text = "กำลังเปิด…";
    action = "รอ…";
  } else if (terminal) {
    state = "ended";
    text = "บันทึกไม่สำเร็จ · ดู terminal";
    tip = "หน้าต่าง terminal ของการบันทึกยังเปิดค้างไว้ให้อ่านข้อความ";
    action = "↻ ลองใหม่";
    enabled = map.mapping;
  } else if (map.saved) {
    state = "run";
    text = "บันทึกแล้ว " + describe(map.file);
    tip = "กดอีกครั้งเมื่อวิ่งเพิ่ม: ไฟล์มีแค่แผนที่ ณ ตอนที่บันทึก";
    enabled = true;
  } else if (map.mapping) {
    state = "off";
    text = map.file.exists() ?
      "รอบนี้ยังไม่ได้บันทึก · ของเดิม " + formatWhen(map.file.lastModified()) :
      "รอบนี้ยังไม่ได้บันทึก";
    enabled = true;
  } else {
    state = "off";
    text = map.file.exists() ? "ล่าสุด " + describe(map.file) : "ยังไม่มีแผนที่ที่บันทึกไว้";
    tip = "เปิด LIO-SAM (ขั้นที่ 2) ก่อน จึงจะมีแผนที่ให้บันทึก";
  }

  save_.status->setText(text);
  save_.status->setToolTip(tip);
  save_.button->setText(action);
  save_.button->setToolTip(enabled ? QString() : tip);
  save_.button->setEnabled(enabled);
  restyle(save_.button, "kind", enabled ? "start" : "");
  for (QWidget * widget :
    std::initializer_list<QWidget *>{save_.card, save_.badge, save_.status})
  {
    restyle(widget, "state", state);
  }
}

void MappingPanel::showEvent(QShowEvent * event)
{
  rviz_common::Panel::showEvent(event);
  if (themed_ && !tabbed_) {
    tabbed_ = true;
    // After this show, when RViz has restored its saved window layout.
    QTimer::singleShot(0, this, &MappingPanel::tabWithDisplays);
  }
}

// In the mapping RViz this panel and Displays share the left dock. As tabs,
// with this one in front, the steps get the whole column and the display tree
// is one click away.
void MappingPanel::tabWithDisplays()
{
  auto * dock = qobject_cast<QDockWidget *>(parentWidget());
  auto * window = dock ? qobject_cast<QMainWindow *>(dock->parentWidget()) : nullptr;
  if (!window || dock->isFloating()) {
    return;
  }
  const Qt::DockWidgetArea area = window->dockWidgetArea(dock);
  for (QDockWidget * other : window->findChildren<QDockWidget *>()) {
    if (other != dock && other->windowTitle() == "Displays" && !other->isFloating() &&
      window->dockWidgetArea(other) == area)
    {
      window->setTabPosition(area, QTabWidget::North);
      window->tabifyDockWidget(other, dock);
      dock->raise();
      return;
    }
  }
}

}  // namespace mapping_rviz_panel

PLUGINLIB_EXPORT_CLASS(mapping_rviz_panel::MappingPanel, rviz_common::Panel)
