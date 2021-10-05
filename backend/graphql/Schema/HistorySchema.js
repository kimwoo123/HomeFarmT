const { gql } = require('apollo-server');

const HistorySchema = gql`
  type Query {
    getHistory: History
  }

  type Mutation {
    createHistory(
      event_time: String,
      event_title: String,
      event_desc: String,
      event_img: String,
      ): History
    deleteHistory(email: String, password: String): Token
  }

  type History {
    historyid: ID!
    envet_time: String!
    envet_title: String!
    envet_desc: String!
    envet_img: String!
  }

`;

module.exports = HistorySchema;
