#!/bin/sh
killall -9 gzserver
killall -9 gzclient
killall -9 roscore
killall -9 rosmaster
killall -9 ray::IDLE
killall -9 ray::RolloutWorker
