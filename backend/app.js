var createError = require('http-errors');
var path = require('path');
var express = require('express');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
const cors = require('cors');
const helmet = require('helmet');
const jwt = require('jsonwebtoken');
const { ApolloServer } = require('apollo-server-express');
const { ApolloServerPluginDrainHttpServer } = require('apollo-server-core');
const http = require('http');
var indexRouter = require('./routes/index');
const UsertypeDefs = require('./graphql/Schema/UserSchema');
const ScheduletypeDefs = require('./graphql/Schema/ScheduleSchema');
const UserResolvers = require('./graphql/Resolvers/UserResolvers');
const ScheduleResolvers = require('./graphql/Resolvers/ScheduleResolvers');
const _ = require('lodash');
const socketIo = require('socket.io')

const typeDefs = [UsertypeDefs, ScheduletypeDefs]
const resolvers = _.merge({}, UserResolvers, ScheduleResolvers)
require('dotenv').config()

const { sequelize } = require('./models')

sequelize.sync({force: false})
.then(()=>{
    console.log('데이터베이스 연결 성공');
})
.catch((err)=>{
    console.error(err);
});

const context = ({ req }) => {
  if (!req.headers.authorization) return { user: null }
  const token = req.headers.authorization.split(' ')[1] || ''
  const user = jwt.verify(token, process.env.SECRET_KEY)
  return { user: user }
}

async function startApolloServer(typeDefs, resolvers) {
  const app = express();
  const httpServer = http.createServer(app);
  const server = new ApolloServer({
    typeDefs: typeDefs,
    resolvers: resolvers,
    introspection: true,
    context: context,
    plugins: [ApolloServerPluginDrainHttpServer({ httpServer })],
  });

  await server.start();
  server.applyMiddleware({
     app,
     path: '/'
  });

  await new Promise(resolve => httpServer.listen({ port: 4000 }, resolve));
  console.log(`🚀 Server ready at http://localhost:4000${server.graphqlPath}`);
}

var app = express();
startApolloServer(typeDefs, resolvers);

app.use(cors());
app.use(helmet());
app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));
app.use('/', indexRouter);

async function startSocketSErver() {
  const app = express();
  const socketServer = http.createServer(app)
  const io = socketIo(socketServer)
  const port = 30001

  socketServer.listen(port, () => {
    console.log('listen on 30001')
  })
}

startSocketSErver()
// Websocket 서버 구동을 위한 서버 코드입니다.

// 노드 로직 순서


// client 경로의 폴더를 지정해줍니다.
const publicPath = path.join(__dirname, "/../client");
app.use(express.static(publicPath));

// 로직 1. WebSocket 서버, WebClient 통신 규약 정의



var fs = require('fs'); // required for file serving

// 로직 2. 포트번호 지정

const roomName = 'team';

// io.on('connection', socket => {
//     socket.join(roomName);

//     // 로직 3. 사용자의 메시지 수신시 WebClient로 메시지 전달
//     socket.on('safety_status', (message) => {
//         console.log(message)
//         console.log(socket)
//         socket.to(roomName).emit('sendSafetyStatus', message);
//     });

//     socket.on('PatrolStatus', (message) => {
//         socket.to(roomName).emit('sendPatrolStatus', message);
//     });

//     socket.on('PatrolOnToServer', (data) => {
//         socket.to(roomName).emit('patrolOn', data);
//         console.log('Patrol On!');
//     });

//     socket.on('PatrolOffToServer', (data) => {
//         socket.to(roomName).emit('patrolOff', data);
//     });

//     socket.on('turnleftToServer', (data) => {
//         socket.to(roomName).emit('turnleft', data);
//     });

//     socket.on('gostraightToServer', (data) => {
//         socket.to(roomName).emit('sendAirConOn', data);
//     });

//     socket.on('turnrightToServer', (data) => {
//         socket.to(roomName).emit('turnright', data);
//     });

//     socket.on('disconnect', () => {
//         console.log('disconnected from server');
//     });

//     // 전달받은 이미지를 jpg 파일로 저장
//     // socket.on('streaming', (message) => {
//     //     socket.to(roomName).emit('sendStreaming', message);
//     //     // console.log(message);
//     //     buffer = Buffer.from(message, "base64");
//     //     fs.writeFileSync(path.join(picPath, "/../client/cam.jpg"), buffer);
//     // });

// })


app.disable('x-powered-by');
module.exports = app;