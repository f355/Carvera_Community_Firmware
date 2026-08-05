#ifndef REMOTEFACTORYPROVIDER_H
#define REMOTEFACTORYPROVIDER_H

#if defined(MACHINE_Z1)

#include "libs/Kernel.h"
#include "modules/communication/RemoteTransfer.h"

class SerialConsole;

class RemoteFactoryProvider : private remote::RecordSink {
 public:
  explicit RemoteFactoryProvider(const FACTORY_SET& current);

  remote::Result fetch(SerialConsole& serial);
  const FACTORY_SET& value() const { return candidate_; }
  bool changed() const;

 private:
  uint32_t accept(uint32_t index, const uint8_t* data, std::size_t size) override;
  bool commit() override;
  void rollback() override;

  FACTORY_SET original_;
  FACTORY_SET candidate_;
};

#endif
#endif
