#ifndef MAPPING_RVIZ_PANEL__MAPPING_PANEL_HPP_
#define MAPPING_RVIZ_PANEL__MAPPING_PANEL_HPP_

#include <sys/types.h>

#include <QElapsedTimer>
#include <QFileInfo>
#include <QRegularExpression>
#include <QString>

#include <vector>

#include <rviz_common/panel.hpp>

class QFrame;
class QLabel;
class QPushButton;
class QShowEvent;

namespace mapping_rviz_panel
{

// One button per step of a mapping run, in the order they are brought up, and
// one that saves the map. Built like cart_rviz_panel in 2Localize_get_path:
// each step runs in its own terminal window, like typing the command by hand,
// and its state is read from the process table every second rather than kept
// in a flag. So the panel stays right when a step is started or stopped from
// an ordinary terminal, dies by itself, or RViz is restarted, and closing
// RViz leaves everything running.
class MappingPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MappingPanel(QWidget * parent = nullptr);

protected:
  void showEvent(QShowEvent * event) override;

private:
  struct Proc
  {
    pid_t pid;
    pid_t pgid;
    QString cmd;  // /proc/<pid>/cmdline with the NULs turned into spaces
  };

  // A process in a preparing step's group that tells what it is waiting for.
  struct Wait
  {
    QRegularExpression process;
    QString text;
  };

  struct Step
  {
    QString key;                  // name of the script the terminal runs
    int number = 0;
    QString name;
    QString label;                // "<number>. <name>", for dialogs and titles
    QString command;              // run from the workspace after sourcing it
    QRegularExpression running;   // a process that means the step is up
    QRegularExpression starting;  // optional: up, but not ready yet
    std::vector<Wait> waits;
    QString device;               // optional: a device node the step needs
    QString confirm_stop;         // non-empty: ask before stopping
    QRegularExpression terminal;  // the bash running this step's script

    QFrame * card = nullptr;
    QLabel * badge = nullptr;
    QLabel * status = nullptr;
    QPushButton * button = nullptr;
    QFrame * segment = nullptr;   // this step's part of the header strip

    QElapsedTimer start_clock;    // valid while waiting for the step to show up
    QElapsedTimer stop_clock;     // restarted on each stop signal sent from here
    int stop_level = 0;           // stop signals sent so far: 1 INT, 2 TERM, 3 KILL
  };

  // The save button. Not a step: the call ends by itself once the map is
  // written, and what it leaves behind is a file, not a process.
  struct Save
  {
    QString command;
    QRegularExpression node;      // LIO-SAM's map node: while it is up there is a map to save
    QRegularExpression calling;   // the service call, there for as long as the map is being written
    QRegularExpression terminal;  // the bash running the save script

    QFrame * card = nullptr;
    QLabel * badge = nullptr;
    QLabel * status = nullptr;
    QPushButton * button = nullptr;

    QElapsedTimer start_clock;    // valid while waiting for the call to show up
  };

  // What the process table and the save folder say about the map right now.
  struct MapState
  {
    bool mapping = false;  // LIO-SAM is up
    bool saving = false;   // a save is in flight
    bool saved = false;    // GlobalMap.pcd was written during this LIO-SAM run
    QFileInfo file;        // GlobalMap.pcd
  };

  void addStep(
    const QString & key, const QString & name, const QString & command,
    const QString & running, const QString & starting = QString());
  void buildUi();
  QFrame * buildCard(Step & step, size_t index);
  QFrame * buildSaveCard();
  void onClicked(size_t index);
  void onSaveClicked();
  void stop(Step & step, const std::vector<Proc> & procs);
  void refresh();
  void refreshSave(const std::vector<Proc> & procs);
  void tabWithDisplays();

  static std::vector<Proc> listProcs();
  bool isUp(const Step & step, const std::vector<Proc> & procs) const;
  bool ownsGroup(const Step & step, const Proc & proc, const std::vector<Proc> & procs) const;
  const Step * ownerOf(const Step & step, const Proc & proc, const std::vector<Proc> & procs) const;
  MapState mapState(const std::vector<Proc> & procs) const;
  QString stopQuestion(const Step & step, const std::vector<Proc> & procs) const;
  bool run(
    const QString & key, const QString & label, const QString & command,
    const QRegularExpression & terminal, const std::vector<Proc> & procs);
  QString writeScript(const QString & key, const QString & label, const QString & command) const;
  bool openTerminal(const QString & title, const QString & script) const;
  Step * find(const QString & key);

  std::vector<Step> steps_;
  Save save_;
  QString script_dir_;
  QLabel * running_chip_ = nullptr;
  QLabel * busy_chip_ = nullptr;
  QLabel * ended_chip_ = nullptr;
  bool themed_ = false;  // the mapping RViz: dark window, tabbed with Displays
  bool tabbed_ = false;
};

}  // namespace mapping_rviz_panel

#endif  // MAPPING_RVIZ_PANEL__MAPPING_PANEL_HPP_
