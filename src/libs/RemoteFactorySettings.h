#ifndef REMOTE_FACTORY_SETTINGS_H
#define REMOTE_FACTORY_SETTINGS_H

#if defined(MACHINE_FAMILY_Z1)

#include "libs/FactorySettings.h"
#include "modules/communication/RemoteTransfer.h"

class SerialConsole;

class RemoteFactorySettings : private remote::RecordSink {
  public:
    explicit RemoteFactorySettings(const FACTORY_SET &current);

    remote::Result fetch(SerialConsole &serial);
    const FACTORY_SET &value() const
    {
        return candidate_;
    }
    bool changed() const;

  private:
    uint32_t accept(uint32_t index, const uint8_t *data, std::size_t size) override;
    bool commit() override;
    void rollback() override;

    FACTORY_SET original_;
    FACTORY_SET candidate_;
};

#endif
#endif
