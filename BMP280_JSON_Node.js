[{"id":"6e2fed4b.325c24","type":"tab","label":"Flow 1","disabled":false,"info":""},{"id":"cf1bcee.62c9b3","type":"mqtt in","z":"6e2fed4b.325c24","name":"","topic":"TempC","qos":"2","broker":"2f70a0d2.a0ff7","x":84.78122329711914,"y":81.9176082611084,"wires":[["dc6fdc66.f71fb","8754c0da.c5f9c","9284055c.420db8"]]},{"id":"7ef09837.92cbe8","type":"mqtt in","z":"6e2fed4b.325c24","name":"","topic":"TempF","qos":"2","broker":"2f70a0d2.a0ff7","x":91.78406143188477,"y":180.9204216003418,"wires":[["2914dd13.33f262","67fdbfea.b0ff1","9284055c.420db8"]]},{"id":"ebbe39a4.ec43c8","type":"mqtt in","z":"6e2fed4b.325c24","name":"","topic":"Pressure","qos":"2","broker":"2f70a0d2.a0ff7","x":102.78975296020508,"y":240.92321968078613,"wires":[["9c7a94d2.676388","91c23ee3.44a0f","9284055c.420db8"]]},{"id":"ffd75aa6.973ad8","type":"mqtt in","z":"6e2fed4b.325c24","name":"","topic":"FAltitude","qos":"2","broker":"2f70a0d2.a0ff7","x":102.79261016845703,"y":304.92326736450195,"wires":[["7c55d2fe.0c519c","73f96b.3b607694","9284055c.420db8"]]},{"id":"8754c0da.c5f9c","type":"ui_chart","z":"6e2fed4b.325c24","name":"","group":"699c61b6.51c8e","order":2,"width":0,"height":0,"label":"C Temp","chartType":"line","legend":"false","xformat":"HH:mm","interpolate":"linear","nodata":"","dot":false,"ymin":"0","ymax":"50","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff8040","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":294.78406143188477,"y":85.7159013748169,"wires":[[],[]]},{"id":"67fdbfea.b0ff1","type":"ui_chart","z":"6e2fed4b.325c24","name":"","group":"699c61b6.51c8e","order":1,"width":0,"height":0,"label":"F temp","chartType":"line","legend":"false","xformat":"HH:mm","interpolate":"linear","nodata":"","dot":false,"ymin":"0","ymax":"50","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#a61e1e","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":341.7925796508789,"y":166.7130241394043,"wires":[[],[]]},{"id":"91c23ee3.44a0f","type":"ui_chart","z":"6e2fed4b.325c24","name":"","group":"6d81d54e.2e9c4c","order":1,"width":0,"height":0,"label":"Pressure","chartType":"bar","legend":"false","xformat":"HH:mm:ss","interpolate":"linear","nodata":"","dot":false,"ymin":"0","ymax":"1000","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":321.79257583618164,"y":245.71016120910645,"wires":[[],[]]},{"id":"7c55d2fe.0c519c","type":"ui_chart","z":"6e2fed4b.325c24","name":"","group":"6d81d54e.2e9c4c","order":2,"width":0,"height":0,"label":"Altitude","chartType":"line","legend":"false","xformat":"HH:mm","interpolate":"linear","nodata":"","dot":false,"ymin":"0","ymax":"1000","removeOlder":1,"removeOlderPoints":"","removeOlderUnit":"3600","cutout":0,"useOneColor":false,"colors":["#1f77b4","#aec7e8","#ff7f0e","#2ca02c","#98df8a","#d62728","#ff9896","#9467bd","#c5b0d5"],"useOldStyle":false,"x":324.7897415161133,"y":298.7101707458496,"wires":[[],[]]},{"id":"dc6fdc66.f71fb","type":"debug","z":"6e2fed4b.325c24","name":"","active":true,"tosidebar":true,"console":false,"tostatus":false,"complete":"false","x":272.7926025390625,"y":52.95738220214844,"wires":[]},{"id":"2914dd13.33f262","type":"debug","z":"6e2fed4b.325c24","name":"","active":true,"tosidebar":true,"console":false,"tostatus":false,"complete":"false","x":327.7897529602051,"y":124.95169448852539,"wires":[]},{"id":"9c7a94d2.676388","type":"debug","z":"6e2fed4b.325c24","name":"","active":true,"tosidebar":true,"console":false,"tostatus":false,"complete":"false","x":287.7926139831543,"y":203.94882774353027,"wires":[]},{"id":"73f96b.3b607694","type":"debug","z":"6e2fed4b.325c24","name":"","active":true,"tosidebar":true,"console":false,"tostatus":false,"complete":"false","x":305.78690338134766,"y":343.9573230743408,"wires":[]},{"id":"9284055c.420db8","type":"join","z":"6e2fed4b.325c24","name":"","mode":"custom","build":"object","property":"payload","propertyType":"msg","key":"topic","joiner":"","joinerType":"str","accumulate":true,"timeout":"4","count":"4","reduceRight":false,"reduceExp":"","reduceInit":"","reduceInitType":"","reduceFixup":"","x":129.7840518951416,"y":401.926100730896,"wires":[["a2ed8c13.c2f8c"]]},{"id":"d2abecc.983171","type":"function","z":"6e2fed4b.325c24","name":"","func":"p = JSON.parse(msg.payload);\nnode.log(typeof p);\nmsg.payload = p;\nreturn msg;","outputs":1,"noerr":0,"x":393.7812309265137,"y":398.9289665222168,"wires":[["c665b986.34e148"]]},{"id":"a2ed8c13.c2f8c","type":"json","z":"6e2fed4b.325c24","name":"","property":"payload","action":"","pretty":false,"x":257.7925796508789,"y":397.92608642578125,"wires":[["d2abecc.983171"]]},{"id":"c665b986.34e148","type":"debug","z":"6e2fed4b.325c24","name":"","active":true,"tosidebar":true,"console":false,"tostatus":false,"complete":"false","x":547.7840805053711,"y":400.9460029602051,"wires":[]},{"id":"2f70a0d2.a0ff7","type":"mqtt-broker","z":"","name":"iot.eclipse.org","broker":"iot.eclipse.org","port":"1883","clientid":"","usetls":false,"compatmode":true,"keepalive":"5","cleansession":true,"birthTopic":"","birthQos":"0","birthPayload":"","closeTopic":"","closeQos":"0","closePayload":"","willTopic":"","willQos":"0","willPayload":""},{"id":"699c61b6.51c8e","type":"ui_group","name":"Group 1","tab":"f4a73006.d45dc","order":1,"disp":true,"width":6},{"id":"6d81d54e.2e9c4c","type":"ui_group","name":"Group 2","tab":"f4a73006.d45dc","order":2,"disp":true,"width":6},{"id":"f4a73006.d45dc","type":"ui_tab","z":"","name":"Altitude","icon":"dashboard","order":3}]