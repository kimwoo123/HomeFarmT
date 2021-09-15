const { gql } = require('apollo-server');

const typeDefs = gql`
  type Query {
    allUser: [User]
  }

  type Mutation {
    signUp(email: String, userid: ID): User
    deleteUser(email: String, userid: ID): User
  }

  type User {
    userid: ID!
    email: String!
  }

`;

module.exports = typeDefs;
