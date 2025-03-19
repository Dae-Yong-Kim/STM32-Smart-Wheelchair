class Graph {
    constructor() {
        this.nodes = new Set();
        this.edges = new Map();
        this.directions = new Map(); // 방향 정보를 저장할 맵 추가
    }

    addNode(node) {
        this.nodes.add(node);
        this.edges.set(node, new Map());
        this.directions.set(node, new Map()); // 방향 정보도 초기화
    }

    addEdge(node1, node2, weight) {
        this.edges.get(node1).set(node2, weight);
        this.edges.get(node2).set(node1, weight);

        // 방향 정보 저장
        this.directions.get(node1).set(node2, new Map());
        this.directions.get(node2).set(node1, new Map());
    }

    addDirection(node1, node2, node3, direction) {
        // 반대 방향 자동 설정
        let oppositeDirection = direction === "좌회전" ? "우회전" : direction === "우회전" ? "좌회전" : "직진";
        this.directions.get(node1).get(node2).set(node3, direction);
        this.directions.get(node3).get(node2).set(node1, oppositeDirection);
    }

    dijkstra(start, end) {
        let distances = new Map();
        let previous = new Map();
        let pq = new Set(this.nodes);

        // 모든 노드의 초기 거리를 무한대로 설정, 시작 노드는 0
        this.nodes.forEach(node => distances.set(node, Infinity));
        distances.set(start, 0);

        while (pq.size) {
            // 현재 가장 작은 거리의 노드 선택
            let current = [...pq].reduce((a, b) => distances.get(a) < distances.get(b) ? a : b);
            pq.delete(current);

            if (current === end) break; // 도착지 도달 시 종료

            // 이웃 노드 업데이트
            this.edges.get(current).forEach((weight, neighbor) => {
                if (!pq.has(neighbor)) return;

                let newDistance = distances.get(current) + weight;
                if (newDistance < distances.get(neighbor)) {
                    distances.set(neighbor, newDistance);
                    previous.set(neighbor, current);
                }
            });
        }

        // 최단 경로 및 방향 생성
        let path = [];
        let directions = [];
        let step = end;

        while (previous.has(step)) {
            let prevStep = previous.get(step);
            path.unshift(step);
            
            // 방향 정보가 존재하면 추가
            if (this.directions.has(prevStep) && this.directions.get(prevStep).has(step)) {
                let nextStep = path.length > 1 ? path[1] : null;
                if (nextStep && this.directions.get(prevStep).get(step).has(nextStep)) {
                    directions.unshift(this.directions.get(prevStep).get(step).get(nextStep));
                } else {
                    //directions.unshift("알 수 없음"); // 방향 정보가 없을 경우 기본값 설정
                }
            } else {
                //directions.unshift("알 수 없음");
            }

            step = prevStep;
        }
        path.unshift(start);

        return { distance: distances.get(end), path, directions };
    }
}

// 그래프 정의
const graph = new Graph();
['A', 'B', 'C', 'D', 'E', 'F'].forEach(node => graph.addNode(node));

graph.addEdge('A', 'C', 1);
graph.addEdge('A', 'E', 1);
graph.addEdge('A', 'F', 3);
graph.addEdge('B', 'C', 1);
graph.addEdge('B', 'F', 3);
graph.addEdge('C', 'D', 1);
graph.addEdge('D', 'E', 2);

graph.addDirection('A', 'C', 'B', "좌회전");
graph.addDirection('A', 'C', 'D', "우회전");
graph.addDirection('A', 'F', 'B', "우회전");
graph.addDirection('A', 'E', 'D', "좌회전");
graph.addDirection('B', 'C', 'D', "직진");
graph.addDirection('C', 'B', 'F', "좌회전");
graph.addDirection('C', 'A', 'F', "우회전");
graph.addDirection('C', 'D', 'E', "우회전");
graph.addDirection('C', 'A', 'E', "좌회전");
graph.addDirection('C', 'A', 'E', "좌회전");
graph.addDirection('E', 'A', 'F', "직진");

// 최단 경로 테스트 (예시코드)
const result = graph.dijkstra('F', 'D');
console.log("최단 거리:", result.distance);
console.log("경로:", result.path.join(" → "));
console.log("방향:", result.directions.join(" → "));
//예상 출력
/*
최단 거리: 10
경로: A → C → D
방향: 직진 → 직진
*/

// 서버에서 사용할 코드
/*
// 출발지와 목적지가 주어지면 다익스트라 실행
let match = receivedData.match(/^(\w)\s*->\s*(\w)$/);
if (match) {
    let start = match[1];
    let end = match[2];

    if (!graph.nodes.has(start) || !graph.nodes.has(end)) {
        socket.write(`잘못된 노드 입력: ${start}, ${end}\r\n`);
        return;
    }

    let result = graph.dijkstra(start, end);
    socket.write(`최단 거리: ${result.distance}, 경로: ${result.path.join(' -> ')}\r\n`);
    return;
}
*/

module.exports = graph;