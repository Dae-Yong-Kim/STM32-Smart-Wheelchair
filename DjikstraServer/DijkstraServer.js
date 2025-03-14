var net = require('net'); // net 모듈 로드

// 클라이언트들을 저장할 배열
let clients = [];

var server = net.createServer((socket) => { // TCP 서버를 만든다.
    console.log('클라이언트 연결됨');

    // 클라이언트 상태를 socket 객체에 저장
    socket.clientState = {
        type: 'APP',  // 클라이언트 유형 (8000, 8100 등)
        buffer: ''   // 수신한 데이터 버퍼
    };

    // 벨트 & 압력 확인 변수
    var hall = 1, force = 1, Emergency_hf = 0, Emergency_hb = 0;

    // 새로 연결된 클라이언트를 clients 배열에 추가
    clients.push(socket);
    clients.forEach(client => {
        console.log(client.clientState.type);
        console.log(client.clientState.buffer);
    });

    // 클라이언트로부터 데이터 수신
    socket.on('data', (data) => {
        console.log('클라이언트로부터 받은 데이터:', data.toString());

        // 클라이언트 상태에 따라 데이터 처리
        if (socket.clientState.type === 'APP') {
            // 처음 접속 시, 클라이언트의 타입을 구분
            let receivedData = data.toString().trim();

            if (receivedData === '8000') {
                socket.clientState.type = 'WheelChair';
                socket.write('WheelChair\r\n');
            } else if (receivedData === '8100') {
                socket.clientState.type = 'LCD';
                socket.write('LCD\r\n');
            }
        } else {
            // 클라이언트의 타입에 맞춰 추가 데이터를 처리
            if (socket.clientState.type === 'WheelChair') {
                socket.clientState.buffer += data.toString();
        
                // Origin 처리
                while (socket.clientState.buffer.startsWith('1') && socket.clientState.buffer.length >= 6) {
                    let message = socket.clientState.buffer.slice(0, 6);
                    socket.clientState.buffer = socket.clientState.buffer.slice(6); // 나머지 버퍼 갱신
                    clients.forEach(client => {
                        if (client.clientState.type === 'APP') {
                            client.write(message);
                            console.log('출발지 (WC -> SERVER -> APP):', message);
                        }
                    });
                }
        
                // Arrive at Destination 처리
                while (socket.clientState.buffer.startsWith('3') && socket.clientState.buffer.length >= 6) {
                    let message = socket.clientState.buffer.slice(0, 6);
                    socket.clientState.buffer = socket.clientState.buffer.slice(6); // 나머지 버퍼 갱신
                    clients.forEach(client => {
                        if (client.clientState.type === 'APP' || client.clientState.type === 'LCD') {
                            client.write(message);
                            console.log('목적지 도착 (WC -> SERVER -> APP, LCD):', message);
                        }
                    });
                }
        
                // Voltage Sensor 처리
                while (socket.clientState.buffer.startsWith('7') && socket.clientState.buffer.length >= 6) {
                    let message = socket.clientState.buffer.slice(0, 6);
                    socket.clientState.buffer = socket.clientState.buffer.slice(6); // 나머지 버퍼 갱신
                    clients.forEach(client => {
                        if (client.clientState.type === 'LCD') {
                            client.write(message);
                            console.log('배터리 데이터 (WC -> SERVER -> LCD):', message);
                        }
                    });
                }
            } else if (socket.clientState.type === 'LCD') {
                socket.clientState.buffer += data.toString();
        
                // Destination 처리
                while (socket.clientState.buffer.startsWith('2') && socket.clientState.buffer.length >= 6) {
                    let message = socket.clientState.buffer.slice(0, 6);
                    socket.clientState.buffer = socket.clientState.buffer.slice(6); // 나머지 버퍼 갱신
                    clients.forEach(client => {
                        if (client.clientState.type === 'WheelChair') {
                            client.write(message);
                            console.log('목적지 (LCD -> SERVER -> WC):', message);
                        }
                    });
                }
        
                // Hall Sensor 처리
                while (socket.clientState.buffer.startsWith('4') && socket.clientState.buffer.length >= 6) {
                    let message = socket.clientState.buffer.slice(0, 6);
                    socket.clientState.buffer = socket.clientState.buffer.slice(6); // 나머지 버퍼 갱신
                    if (message === '4000\r\n') { // unbuckle
                        hall = 1;
                        if ((force === 1) && (Emergency_hf === 0)) { // Emergency
                            Emergency_hf = 1;
                            clients.forEach(client => {
                                client.write('0000\r\n');
                                console.log('Emergency');
                            });
                        }
                    } else { // buckle
                        hall = 0;
                        if ((force === 0) && (Emergency_hb === 0)) { // All Right
                            clients.forEach(client => {
                                client.write('0300\r\n');
                                console.log('All Right');
                            });
                        }
                    }
                    clients.forEach(client => {
                        if (client.clientState.type === 'WheelChair') {
                            client.write(message);
                            console.log('안전벨트 (LCD -> SERVER -> WC):', message);
                        }
                    });
                }
        
                // Force Sensor 처리
                while (socket.clientState.buffer.startsWith('5') && socket.clientState.buffer.length >= 6) {
                    let message = socket.clientState.buffer.slice(0, 6);
                    socket.clientState.buffer = socket.clientState.buffer.slice(6); // 나머지 버퍼 갱신
                    if (message === '5000\r\n') { // stand
                        force = 1;
                        if (hall === 1) { // Emergency
                            Emergency_hf = 1;
                            clients.forEach(client => {
                                client.write('0000\r\n');
                                console.log('Emergency');
                            });
                        }
                    } else { // sit
                        force = 0;
                        if (Emergency_hf === 1) {
                            Emergency_hf = 0;
                            clients.forEach(client => {
                                client.write('0100\r\n');
                                console.log('Emergency_hf 끝');
                            });
                        }
                        if ((hall === 0) && (Emergency_hb === 0)) { // All Right
                            clients.forEach(client => {
                                client.write('0300\r\n');
                                console.log('All Right');
                            });
                        }
                    }
                }
        
                // Heartbeat Sensor 처리
                while (socket.clientState.buffer.startsWith('6') && socket.clientState.buffer.length >= 6) {
                    let message = socket.clientState.buffer.slice(0, 6);
                    socket.clientState.buffer = socket.clientState.buffer.slice(6); // 나머지 버퍼 갱신
                    if (message === '6200\r\n') { // Emergency
                        Emergency_hb = 1;
                        clients.forEach(client => {
                            client.write('0000\r\n');
                            console.log('Emergency');
                        });
                    } else if (message === '6100\r\n') {
                        Emergency_hb = 0;
                        clients.forEach(client => {
                            client.write('0200\r\n');
                            console.log('Emergency_hb 끝');
                        });
                        if ((force === 0) && (hall === 0)) { // All Right
                            clients.forEach(client => {
                                client.write('0300\r\n');
                                console.log('All Right');
                            });
                        }
                    }
                    clients.forEach(client => {
                        if (client.clientState.type === 'WheelChair') {
                            client.write(message);
                            console.log('심박 데이터 (LCD -> SERVER -> WC):', message);
                        }
                    });
                }
            }
        }
        
    });

  // 연결 종료 전에 클라이언트 종류에 따라 종료 처리
  socket.on('end', () => {
    if (socket.clientState.type === 'WheelChair') {
      console.log('WheelChair 클라이언트와의 연결 종료');
    } else if (socket.clientState.type === 'LCD') {
      console.log('LCD 클라이언트와의 연결 종료');
    } else {
      console.log('APP 클라이언트 종료');
    }
    console.log('최종 버퍼 내용:', socket.clientState.buffer);
    
    let index = clients.indexOf(socket);
    if (index !== -1) {
        clients.splice(index, 1); // 해당 클라이언트(socket)를 배열에서 제거
        console.log('클라이언트가 제거되었습니다.');
    }
  });

  // 에러 처리
  socket.on('error', (err) => {
    console.log('소켓 에러:', err);
    
    let index = clients.indexOf(socket);
    if (index !== -1) {
        clients.splice(index, 1); // 해당 클라이언트(socket)를 배열에서 제거
        console.log('클라이언트가 제거되었습니다.');
    }
  });
});

server.on('error', (err) => { // 네트워크 에러 처리
  console.log('서버 에러:', err);
    
  let index = clients.indexOf(socket);
  if (index !== -1) {
      clients.splice(index, 1); // 해당 클라이언트(socket)를 배열에서 제거
      console.log('클라이언트가 제거되었습니다.');
  }
});

server.listen(3000, () => {
  console.log('서버가 시작되었습니다. 주소:', server.address());
});
