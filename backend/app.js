var createError = require('http-errors');
var express = require('express');
var path = require('path');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
const cors = require('cors')
const { ApolloServer } = require('apollo-server-express');
const { ApolloServerPluginDrainHttpServer } = require('apollo-server-core')
const http = require('http')
var indexRouter = require('./routes/index');
const UsertypeDefs = require('./graphql/Schema/UserSchema');
const ScheduletypeDefs = require('./graphql/Schema/ScheduleSchema');
const UserResolvers = require('./graphql/Resolvers/UserResolvers');
const ScheduleResolvers = require('./graphql/Resolvers/ScheduleResolvers');
const _ = require('lodash');

const typeDefs = [UsertypeDefs, ScheduletypeDefs]
const resolvers = _.merge({}, UserResolvers, ScheduleResolvers)

async function startApolloServer(typeDefs, resolvers) {
  const app = express();
  const httpServer = http.createServer(app);

  const server = new ApolloServer({
    typeDefs: typeDefs,
    resolvers: resolvers,
    introspection: true,
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

module.exports = app;
