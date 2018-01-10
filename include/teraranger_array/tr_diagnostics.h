#pragma once
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <multiple_asynchronous_timers/AsyncTimerArray.h>

class RangeDiag : public diagnostic_updater::DiagnosticTask
{
public:
  RangeDiag();
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void setAta(AsyncTimerArray* ata);

private:
  AsyncTimerArray* ata_ = nullptr;
};
