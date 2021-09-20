var createError = require('http-errors');
var path = require('path');
var express = require('express');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
const cors = require('cors')
const helmet = require('helmet')
const { ApolloServer } = require('apollo-server-express');
const { ApolloServerPluginDrainHttpServer } = require('apollo-server-core')
const http = require('http')
var indexRouter = require('./routes/index');
const { createStore } = require('./graphql/utils');
const UsertypeDefs = require('./graphql/Schema/UserSchema');
const ScheduletypeDefs = require('./graphql/Schema/ScheduleSchema');
const UserResolvers = require('./graphql/Resolvers/UserResolvers');
const ScheduleResolvers = require('./graphql/Resolvers/ScheduleResolvers');
const _ = require('lodash');

const typeDefs = [UsertypeDefs, ScheduletypeDefs]
const resolvers = _.merge({}, UserResolvers, ScheduleResolvers)

const store = createStore();

const context = ({ req }) => {
  const token = req.headers.authorization || ''
  const email = Buffer.from(token, 'base64').toString('ascii')
  if (email) {
    const user = store.users.findByPk({ where: { email }})
  }
  return { email }
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

app.use(function(req, res, next) {
  next(createError(404));
});

app.use(function(err, req, res, next) {
  res.locals.message = err.message;
  res.locals.error = req.app.get('env') === 'development' ? err : {};
  res.status(err.status || 500);
  res.render('error');
});

app.disable('x-powered-by');

module.exports = app;
