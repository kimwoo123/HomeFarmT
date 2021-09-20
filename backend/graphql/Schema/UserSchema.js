const { gql } = require('apollo-server');

const UserSchema = gql`
  type Query {
    allUser: [User]
    findUser(password: String): User
  }

  type Mutation {
    signUp(email: String, password: String): User
    login(email: String, password: String): Token
    deleteUser(email: String, password: String): User
  }

  type User {
    email: String!
    password: String!
  }

  type Token {
    email: String!
    password: String!
    token: String
    message: String
  }

`;

module.exports = UserSchema;
