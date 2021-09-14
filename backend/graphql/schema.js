const { gql } = require('apollo-server');

const typeDefs = gql`
  type Query {
    me: User
  }

  type Mutation {
    signup(email: String, userid: ID): User
    deleteUser(email: String, userid: ID): User
  }

  type User {
    userid: ID!
    email: String!
  }

`;

module.exports = typeDefs;
