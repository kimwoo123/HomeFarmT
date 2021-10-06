const { gql } = require('apollo-server');

const UserSchema = gql`
  type Query {
    allUser: [User]
    findUser: UserInfo
  }

  type Mutation {
    signUp(email: String, password: String): User
    login(email: String, password: String): Token
    updateUser(password: String, turtlebot: String, region: String): UserInfo
    deleteUser(email: String, password: String): User
  }

  type User {
    email: String!
    password: String!
  }

  type UserInfo {
    email: String!
    password: String!
    hashid: String
    map: String
    route1: String
    route2: String
    route3: String
    region: String
    turtlebot: String
    patrol: String
  }

  type Token {
    email: String!
    password: String!
    token: String
    message: String
  }

`;

module.exports = UserSchema;
