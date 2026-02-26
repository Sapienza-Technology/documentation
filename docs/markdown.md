
# Markdown for dummies

Here is a [Demo page](https://docs.owid.io/projects/etl/guides/demo/) for all Zensical features

esempio pazzo

### Contenuto

=== "Testo pazzo"

    semplice testo

=== "Lista non ordinata"

    * suca
    * pagliaccio
    * pazzo

=== "Lista ordinata"

    1. Primo
    2. Secondo
    3. Terzo

!!! note "SEI PAZZO"

    questa è una nota pazza sgravata

???+ info "Questa nota pazza si piegha"

    guarda questo [link](https://squidfunk.github.io/mkdocs-material/reference/admonitions/?h=ad).


## Project layout

    zensical.toml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.

```py title="titolo.py" linenums="15"
def puzza():

    return 
```

## Headers
```
# H1 Header
## H2 Header
### H3 Header
#### H4 Header
##### H5 Header
###### H6 Header
```

## Text formatting
```
**bold text**
*italic text*
***bold and italic***
~~strikethrough~~
==marked text==
^^underline text^^
`inline code`
```


## Links and images
```
[Link text](https://example.com)
[Link with title](https://example.com "Hover title")
![Alt text](image.jpg)
![Image with title](image.jpg "Image title")
```

## Lists
```
Unordered:
- Item 1
- Item 2
  - Nested item

Ordered:
1. First item
2. Second item
3. Third item
```

## Definition lists
`Definition list`

:   This is the first definition
:   This is the second definition

## Blockquotes
```
> This is a blockquote
> Multiple lines
>> Nested quote
```

## Code blocks
````
```javascript
function hello() {
  console.log("Hello, world!");
}
```
````

## Tables
```
| Header 1 | Header 2 | Header 3 |
|----------|----------|----------|
| Row 1    | Data     | Data     |
| Row 2    | Data     | Data     |
```

## Horizontal rule
```
---
or
***
or
___
```

## Task lists
```
- [x] Completed task
- [ ] Incomplete task
- [ ] Another task
```

## Escaping characters
```
Use backslash to escape: \* \_ \# \`
```

## Line breaks
```
End a line with two spaces  
to create a line break.

Or use a blank line for a new paragraph.
```
## Flowcharts
``` mermaid
graph LR
  A[Start] --> B{Error?};
  B -->|Yes| C[Hmm...];
  C --> D[Debug];
  D --> B;
  B ---->|No| E[Yay!];
```

## Sequence diagrams
``` mermaid
sequenceDiagram
  autonumber
  Alice->>John: Hello John, how are you?
  loop Healthcheck
      John->>John: Fight against hypochondria
  end
  Note right of John: Rational thoughts!
  John-->>Alice: Great!
  John->>Bob: How about you?
  Bob-->>John: Jolly good!
```

## Use state diagrams
``` mermaid
stateDiagram-v2
  state fork_state <<fork>>
    [*] --> fork_state
    fork_state --> State2
    fork_state --> State3

    state join_state <<join>>
    State2 --> join_state
    State3 --> join_state
    join_state --> State4
    State4 --> [*]
```

## Class diagrams
``` mermaid
classDiagram
  Person <|-- Student
  Person <|-- Professor
  Person : +String name
  Person : +String phoneNumber
  Person : +String emailAddress
  Person: +purchaseParkingPass()
  Address "1" <-- "0..1" Person:lives at
  class Student{
    +int studentNumber
    +int averageMark
    +isEligibleToEnrol()
    +getSeminarsTaken()
  }
  class Professor{
    +int salary
  }
  class Address{
    +String street
    +String city
    +String state
    +int postalCode
    +String country
    -validate()
    +outputAsLabel()
  }
```