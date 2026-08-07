#ifndef REMOTECONFIGSOURCE_H
#define REMOTECONFIGSOURCE_H

#include "libs/ConfigSource.h"
#include "modules/communication/RemoteTransfer.h"

class ConfigCache;
class SerialConsole;

class RemoteConfigSource : public ConfigSource, private remote::RecordSink {
 public:
  explicit RemoteConfigSource(SerialConsole& serial);

  void transfer_values_to_cache(ConfigCache* cache) override;
  bool is_named(uint16_t check_sum) override;
  bool write(std::string setting, std::string value) override;
  std::string read(uint16_t check_sums[3]) override;

 private:
  uint32_t accept(uint32_t index, const uint8_t* data, std::size_t size) override;
  void rollback() override;

  SerialConsole& serial_;
  ConfigCache* active_cache_ = nullptr;
  uint32_t accepted_records_ = 0;
};

#endif
