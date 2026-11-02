"""Capture real USB frames and record reproducible transport/processing metrics.

Close the dashboard first. No automatic recovery: a failed run is retained as
failed, so transport faults cannot be hidden by resetting the sensor.
"""
import argparse
import struct
import json
from pathlib import Path
import statistics
import time
from usb_source import USBSource
from processing import Processor

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--usb',default='auto')
    parser.add_argument('--frames',type=int,default=100)
    parser.add_argument('--temporal',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.frames<1:parser.error('--frames must be positive')
    args.output.mkdir(parents=True,exist_ok=False)
    source=USBSource(args.usb)
    processor=None;records=[];imu=[];error=None
    started=time.monotonic()
    try:
        processor=Processor(source.calibration,temporal=args.temporal)
        if source.calibration:(args.output/'sensor-calibration.bin').write_bytes(source.calibration)
        source.on_imu=lambda sample:imu.append(sample) if not imu or imu[-1]['sequence']!=sample['sequence'] else None
        with (args.output/'index.jsonl').open('w') as index:
            for i in range(args.frames):
                frame,raw=source.read()
                processor.process(frame,raw)
                filename=f'frame-{i:05}.bin'
                (args.output/filename).write_bytes(raw)
                record={key:frame[key] for key in ('sequence','sensor_counter','capture_ms','delivery_ms',
                    'checksum','checksum_ok','frame_error_code','frame_error_flags','processing','mailbox','received_at')}
                record['file']=filename
                records.append(record);index.write(json.dumps(record)+'\n');index.flush()
                if frame['frame_error_code'] or frame['frame_error_flags']:
                    raise RuntimeError('Sensor reported a frame error; retained for diagnosis')
    except Exception as exc:
        error=str(exc)
        raise
    finally:
        elapsed=time.monotonic()-started
        timing=None
        if not error and (source.diagnostics or {}).get('version',1)>=2:
            source.stop_stream()
            data=source.read_info(b'D',b'L9D1',64)
            source.diagnostics=dict(zip(source.diagnostics,struct.unpack('<16I',data)))
            timing=dict(zip(('start_ms','trigger_ms','wait_ms','read_ms','publish_ms','exposure_ms','transfer_hz','read_chunk_bytes'),struct.unpack('<8I',source.read_info(b'T',b'L9T1',32))))
        summary={'requested_frames':args.frames,'received_frames':len(records),'error':error,
                 'seconds':elapsed,'fps':len(records)/elapsed,'imu_unique_samples':len(imu),
                 'imu_unique_hz':len(imu)/elapsed,'bus_diagnostics':source.diagnostics,
                 'calibration_available':source.calibration is not None,'acquisition_timing':timing,
                 'median_capture_ms':statistics.median(r['capture_ms'] for r in records) if records else None,
                 'median_delivery_ms':statistics.median(r['delivery_ms'] for r in records) if records else None,
                 'median_processing_ms':statistics.median(r['processing']['milliseconds'] for r in records) if records else None}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        (args.output/'imu.json').write_text(json.dumps(imu))
        if processor:processor.close()
        source.close()
        print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
