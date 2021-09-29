var createError = require('http-errors');
var path = require('path');
var express = require('express');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
const cors = require('cors')
const helmet = require('helmet')
const jwt = require('jsonwebtoken')
const { ApolloServer } = require('apollo-server-express');
const { ApolloServerPluginDrainHttpServer } = require('apollo-server-core')
const http = require('http')
var indexRouter = require('./routes/index');
const UsertypeDefs = require('./graphql/Schema/UserSchema');
const ScheduletypeDefs = require('./graphql/Schema/ScheduleSchema');
const UserResolvers = require('./graphql/Resolvers/UserResolvers');
const ScheduleResolvers = require('./graphql/Resolvers/ScheduleResolvers');
const _ = require('lodash');
const httpServer = http.createServer(app)

const typeDefs = [UsertypeDefs, ScheduletypeDefs]
const resolvers = _.merge({}, UserResolvers, ScheduleResolvers)
require('dotenv').config()

const context = ({ req }) => {
  if (!req.headers.authorization) return { user: null }
  const token = req.headers.authorization.split(' ')[1] || ''
  const user = jwt.verify(token, process.env.SECRET_KEY)
  return { user: user }
}

async function startApolloServer(typeDefs, resolvers) {
  const app = express();
  

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

// client 경로의 폴더를 지정해줍니다.
const publicPath = path.join(__dirname, "/../client");

app.use(express.static(publicPath));

// 로직 1. WebSocket 서버, WebClient 통신 규약 정의
var fs = require('fs'); // required for file serving

// 로직 2. 포트번호 지정
app.disable('x-powered-by');

module.exports = app;