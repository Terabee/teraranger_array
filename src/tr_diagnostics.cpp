#include <teraranger_array/tr_diagnostics.h>

RangeDiag::RangeDiag() : DiagnosticTask("Task to check if ranges are nan.")
{}

void RangeDiag::setAta(AsyncTimerArray* ata)
{
  ata_ = ata;
}

void RangeDiag::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (ata_->any_timer_expired())
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Some required sensors have been nan for longer than allowed");
  }
  else if (ata_->any_timer_running())
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Some required sensors have been giving nan");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Required sensors are OK");
  }
}
