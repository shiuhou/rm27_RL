"""Latch a frame change after registration until a new registration is explicit."""


class RegisteredEpochContract:
    def __init__(self,contract,vio,record):
        self.contract,self.vio,self.record=contract,vio,record
        self.frame_changed=False

    def estimate(self,row):
        if self.frame_changed:
            raise ValueError('Native localization epoch requires new registration')
        estimate=self.contract.estimate(row)
        adapter=self.vio.adapter
        previous=adapter.previous if adapter is not None else None
        if previous is not None and estimate.localization_epoch!=previous.localization_epoch:
            self.frame_changed=True
            self.record(dict(event='registered_localization_epoch_changed',
                registered_epoch=previous.localization_epoch,observed_epoch=estimate.localization_epoch,
                source_time_ns=estimate.source_time.raw_value,
                reason='Existing rigid frame registration is not evidence for the new epoch'))
            raise ValueError('Native localization epoch requires new registration')
        return estimate
