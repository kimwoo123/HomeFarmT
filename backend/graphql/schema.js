const { gql } = require('apollo-server');

const typeDefs = gql`
  type Query {
    allUser: [User]
    findUser(password: String): User
  }

  type Mutation {
    signUp(email: String, password: String): User
    login(email: String, password: String): User
    deleteUser(email: String, password: String): User
  }

  type User {
    email: String!
    password: String!
  }

`;

module.exports = typeDefs;
